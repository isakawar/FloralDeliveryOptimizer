import os
from dotenv import load_dotenv

load_dotenv()

API_KEY = os.getenv('API_KEY')
BASE_ADDRESS = os.getenv('BASE_ADDRESS')

# Максимальна кількість замовлень на одного кур'єра
MAX_ORDERS_PER_COURIER = 6 