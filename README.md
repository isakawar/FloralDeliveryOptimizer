# Генератор оптимального маршруту (Flask + Docker)

## Налаштування змінних середовища

Створіть файл `.env` у корені проєкту:
```
API_KEY=your_google_maps_api_key_here
BASE_ADDRESS=Київ, вул. Нагірна, 18/16
```

## Швидкий старт у Docker

1. Побудуйте образ:
   ```bash
   docker build -t route-planner .
   ```
2. Запустіть контейнер:
   ```bash
   docker run -p 8080:8080 route-planner
   ```
3. Відкрийте у браузері: [http://localhost:8080](http://localhost:8080)

---

- Для роботи потрібен доступ до інтернету (Google Maps API)
- Ваш API-ключ та адресу бази задайте у .env 