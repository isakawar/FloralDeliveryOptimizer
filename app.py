from flask import Flask, render_template, request, redirect, url_for, send_from_directory
import pandas as pd
import os
import tempfile
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import requests
from config import API_KEY, BASE_ADDRESS, MAX_ORDERS_PER_COURIER
# ... імпортуйте ваші функції для розрахунку маршруту тут (наприклад, get_coordinates, get_distance_matrix, solve_tsp)

# --- Google Maps API helpers ---
def get_coordinates(address, api_key=API_KEY):
    print(f"DEBUG: Запит координат для адреси: {address}")
    url = 'https://maps.googleapis.com/maps/api/geocode/json'
    params = {'address': address, 'key': api_key}
    print(f"DEBUG: URL: {url}")
    print(f"DEBUG: params: {params}")
    response = requests.get(url, params=params)
    print(f"DEBUG: status_code: {response.status_code}")
    print(f"DEBUG: response.text: {response.text}")
    data = response.json()
    if data['status'] == 'OK':
        location = data['results'][0]['geometry']['location']
        print(f"DEBUG: Знайдено координати: {location}")
        return location['lat'], location['lng']
    else:
        print(f"ERROR: Не вдалося знайти координати для адреси: {address}, статус: {data['status']}")
        raise Exception(f"Не вдалося знайти координати для адреси: {address}")

def get_distance_matrix(locations, api_key=API_KEY):
    n = len(locations)
    distance_matrix = [[0]*n for _ in range(n)]
    time_matrix = [[0]*n for _ in range(n)]
    for i in range(n):
        origins = f"{locations[i][0]},{locations[i][1]}"
        destinations = "|".join([f"{lat},{lng}" for lat, lng in locations])
        url = 'https://maps.googleapis.com/maps/api/distancematrix/json'
        params = {
            'origins': origins,
            'destinations': destinations,
            'key': api_key,
            'mode': 'driving',
            'language': 'uk',
            'units': 'metric'
        }
        response = requests.get(url, params=params)
        data = response.json()
        if data['status'] != 'OK':
            print('Distance Matrix API error:', data)
            raise Exception(f"Помилка Distance Matrix API: {data['status']}")
        row = data['rows'][0]['elements']
        for j, elem in enumerate(row):
            if elem['status'] == 'OK':
                distance_matrix[i][j] = elem['distance']['value']
                time_matrix[i][j] = elem['duration']['value']
            else:
                distance_matrix[i][j] = float('inf')
                time_matrix[i][j] = float('inf')
    return time_matrix, distance_matrix

def minutes_to_hhmm(minutes):
    h = int(minutes // 60)
    m = int(minutes % 60)
    return f"{h:02d}:{m:02d}"

app = Flask(__name__)

@app.route('/', methods=['GET', 'POST'])
def index():
    result = None
    error = None
    total_distance = 0
    if request.method == 'POST':
        file = request.files.get('file')
        couriers = int(request.form.get('couriers', 1))
        departure_time_str = request.form.get('departure_time', '08:00')
        # Перетворюємо час відправлення у секунди від півночі
        h, m = map(int, departure_time_str.split(':'))
        departure_time_sec = h * 3600 + m * 60
        if file and file.filename.endswith('.csv'):
            try:
                temp = tempfile.NamedTemporaryFile(delete=False, suffix='.csv')
                file.save(temp.name)
                orders = pd.read_csv(temp.name)
                addresses = orders['address'].tolist()
                base_coords = get_coordinates(BASE_ADDRESS)
                print('BASE:', BASE_ADDRESS, '->', base_coords)
                coords = []
                for addr in addresses:
                    try:
                        coord = get_coordinates(addr)
                        print(addr, '->', coord)
                        coords.append(coord)
                    except Exception as e:
                        print(f'Не вдалося знайти координати для адреси: {addr}. Помилка: {e}')
                        coords.append((None, None))
                all_points = [base_coords] + coords
                time_matrix, distance_matrix = get_distance_matrix(all_points)
                # --- Формуємо часові вікна ---
                def parse_time_window(row):
                    if pd.isna(row['time_window_start']) or pd.isna(row['time_window_end']):
                        return (0, 24*3600)  # без обмежень
                    h1, m1 = map(int, row['time_window_start'].split(':'))
                    h2, m2 = map(int, row['time_window_end'].split(':'))
                    return (h1*3600 + m1*60, h2*3600 + m2*60)
                time_windows = [(departure_time_sec, 24*3600)]  # для бази
                for _, row in orders.iterrows():
                    tw = parse_time_window(row)
                    time_windows.append(tw)
                def solve_vrp(distance_matrix, time_matrix, num_vehicles, max_orders_per_courier, time_windows):
                    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicles, 0)
                    routing = pywrapcp.RoutingModel(manager)
                    def distance_callback(from_index, to_index):
                        from_node = manager.IndexToNode(from_index)
                        to_node = manager.IndexToNode(to_index)
                        return int(distance_matrix[from_node][to_node])
                    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
                    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
                    def demand_callback(from_index):
                        from_node = manager.IndexToNode(from_index)
                        return 0 if from_node == 0 else 1
                    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
                    routing.AddDimensionWithVehicleCapacity(
                        demand_callback_index,
                        0,
                        [max_orders_per_courier]*num_vehicles,
                        True,
                        'Capacity')
                    # --- Додаємо часові вікна ---
                    def time_callback(from_index, to_index):
                        from_node = manager.IndexToNode(from_index)
                        to_node = manager.IndexToNode(to_index)
                        return int(time_matrix[from_node][to_node])
                    time_callback_index = routing.RegisterTransitCallback(time_callback)
                    routing.AddDimension(
                        time_callback_index,
                        30*60,  # запас (30 хв)
                        24*3600,  # максимальний час маршруту
                        False,
                        'Time')
                    time_dimension = routing.GetDimensionOrDie('Time')
                    for location_idx, (open_time, close_time) in enumerate(time_windows):
                        time_dimension.CumulVar(location_idx).SetRange(open_time, close_time)
                    # Встановлюємо час старту для кожного кур'єра
                    for vehicle_id in range(num_vehicles):
                        index = routing.Start(vehicle_id)
                        time_dimension.CumulVar(index).SetValue(departure_time_sec)
                    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
                    search_parameters.first_solution_strategy = (
                        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
                    solution = routing.SolveWithParameters(search_parameters)
                    routes = []
                    if solution:
                        for vehicle_id in range(num_vehicles):
                            index = routing.Start(vehicle_id)
                            route = []
                            route_distance = 0
                            route_times = []
                            acc_time = departure_time_sec
                            while not routing.IsEnd(index):
                                node = manager.IndexToNode(index)
                                next_index = solution.Value(routing.NextVar(index))
                                if node != 0:
                                    route.append(node)
                                    # Додаємо час до цієї точки
                                    acc_time = solution.Value(time_dimension.CumulVar(index))
                                    route_times.append(acc_time)
                                # Додаємо відстань до наступної точки
                                if not routing.IsEnd(next_index):
                                    route_distance += distance_matrix[node][manager.IndexToNode(next_index)]
                                index = next_index
                            routes.append({'route': route, 'distance': route_distance, 'times': route_times})
                    return routes
                routes = solve_vrp(distance_matrix, time_matrix, couriers, MAX_ORDERS_PER_COURIER, time_windows)
                result = []
                total_distance = 0
                for i, route_info in enumerate(routes):
                    route = route_info['route']
                    route_addresses = [addresses[idx-1] for idx in route if idx > 0]
                    courier_distance = route_info['distance'] / 1000  # метри -> км
                    courier_times = []
                    for t, idx in zip(route_info['times'], route):
                        if idx > 0:
                            tw = time_windows[idx]
                            in_window = tw[0] <= t <= tw[1]
                            t_min = t // 60
                            time_str = minutes_to_hhmm(t_min)
                            window_str = f"{minutes_to_hhmm(tw[0]//60)}-{minutes_to_hhmm(tw[1]//60)}"
                            courier_times.append({'time': t_min, 'in_window': in_window, 'window': (tw[0]//60, tw[1]//60), 'time_str': time_str, 'window_str': window_str})
                    total_distance += courier_distance
                    result.append({'courier': i+1, 'addresses': route_addresses, 'distance': courier_distance, 'times': courier_times})
                os.unlink(temp.name)
            except Exception as e:
                error = f'Помилка обробки файлу: {e}'
        else:
            error = 'Будь ласка, завантажте CSV-файл.'
        if result is not None and len(result) == 0:
            error = 'Не вдалося знайти жодного маршруту. Можливо, для деяких замовлень часові вікна вже минули або неможливо знайти маршрут з урахуванням побажань клієнтів.'
    print('DEBUG result:', result)
    print('DEBUG error:', error)
    print('DEBUG total_distance:', total_distance)
    return render_template('index.html', result=result, error=error, total_distance=total_distance)

@app.route('/static/<path:filename>')
def static_files(filename):
    return send_from_directory('static', filename)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000) 