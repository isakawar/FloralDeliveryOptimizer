import pandas as pd
import requests
from config import API_KEY, BASE_ADDRESS, MAX_ORDERS_PER_COURIER
import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# 1. Зчитування замовлень
ORDERS_FILE = 'orders.csv'
orders = pd.read_csv(ORDERS_FILE)

# 2. Отримання координат адрес

def get_coordinates(address):
    url = f'https://maps.googleapis.com/maps/api/geocode/json'
    params = {'address': address, 'key': API_KEY}
    response = requests.get(url, params=params)
    data = response.json()
    if data['status'] == 'OK':
        location = data['results'][0]['geometry']['location']
        return location['lat'], location['lng']
    else:
        raise Exception(f"Не вдалося знайти координати для адреси: {address}")

# Отримуємо координати для бази
base_coords = get_coordinates(BASE_ADDRESS)

# Отримуємо координати для всіх замовлень
orders['coords'] = orders['address'].apply(get_coordinates)

# 3. Формування матриці відстаней/часу

def get_distance_matrix(locations, api_key):
    """
    locations: список (lat, lng)
    Повертає матриці часу (секунди) та відстані (метри) між усіма точками
    """
    n = len(locations)
    time_matrix = np.zeros((n, n))
    distance_matrix = np.zeros((n, n))
    
    # Google API дозволяє максимум 25 origins/destinations за раз
    # Тому робимо запити по частинах
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
            raise Exception(f"Помилка Distance Matrix API: {data['status']}")
        row = data['rows'][0]['elements']
        for j, elem in enumerate(row):
            if elem['status'] == 'OK':
                time_matrix[i, j] = elem['duration']['value']
                distance_matrix[i, j] = elem['distance']['value']
            else:
                time_matrix[i, j] = float('inf')
                distance_matrix[i, j] = float('inf')
    return time_matrix, distance_matrix

# Формуємо список усіх точок: база + адреси замовлень
all_points = [base_coords] + list(orders['coords'])

# Отримуємо матриці часу та відстані
print('Отримання матриці відстаней/часу...')
time_matrix, distance_matrix = get_distance_matrix(all_points, API_KEY)
print('Готово!')

# 4. Оптимізація маршрутів для кур'єрів

# Підготовка даних для OR-Tools
num_orders = len(orders)
num_couriers = int(np.ceil(num_orders / MAX_ORDERS_PER_COURIER))

# Часові вікна (у секундах від 8:00 ранку)
def parse_time_window(row):
    base_time = 8 * 3600  # 8:00 ранку
    if pd.isna(row['time_window_start']) or pd.isna(row['time_window_end']):
        return (0, 24*3600)  # без обмежень
    h1, m1 = map(int, row['time_window_start'].split(':'))
    h2, m2 = map(int, row['time_window_end'].split(':'))
    return (h1*3600 + m1*60 - base_time, h2*3600 + m2*60 - base_time)

time_windows = [(0, 24*3600)]  # для бази
for _, row in orders.iterrows():
    time_windows.append(parse_time_window(row))

data = {}
data['time_matrix'] = time_matrix.astype(int).tolist()
data['num_vehicles'] = num_couriers
data['depot'] = 0
# Кожен кур'єр може взяти не більше MAX_ORDERS_PER_COURIER
order_demands = [0] + [1]*num_orders
vehicle_capacities = [MAX_ORDERS_PER_COURIER]*num_couriers

data['demands'] = order_demands

data['vehicle_capacities'] = vehicle_capacities
data['time_windows'] = time_windows

# OR-Tools: створення маршрутизатора
def create_routing_model(data):
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Додаємо обмеження по часу
    routing.AddDimension(
        transit_callback_index,
        30*60,  # дозволений запас (30 хв)
        24*3600,  # максимальний час маршруту
        False,
        'Time')
    time_dimension = routing.GetDimensionOrDie('Time')
    for location_idx, (open_time, close_time) in enumerate(data['time_windows']):
        time_dimension.CumulVar(location_idx).SetRange(open_time, close_time)

    # Обмеження по кількості замовлень
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],
        True,  # start cumul to zero
        'Capacity')

    return routing, manager

routing, manager = create_routing_model(data)

# Пошук рішення
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
search_parameters.time_limit.seconds = 30

solution = routing.SolveWithParameters(search_parameters)

# Вивід результату
if solution:
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route = []
        total_time = 0
        total_distance = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            if node_index != 0:
                order = orders.iloc[node_index-1]
                route.append(order['address'])
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            if not routing.IsEnd(index):
                total_time += data['time_matrix'][manager.IndexToNode(previous_index)][manager.IndexToNode(index)]
                total_distance += distance_matrix[manager.IndexToNode(previous_index)][manager.IndexToNode(index)]
        if route:
            print(f'\nКур\'єр {vehicle_id+1}:')
            for i, addr in enumerate(route, 1):
                print(f'  {i}. {addr}')
            print(f'  Загальний час: {int(total_time//60)} хв, відстань: {int(total_distance/1000)} км')
else:
    print('Не вдалося знайти рішення для маршруту!')

# 5. Вивід результату
# (буде реалізовано далі) 