from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.chrome.service import Service
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC

# Path to ChromeDriver
CHROMEDRIVER_PATH = "/usr/local/bin/chromedriver"

def fetch_exchange_rate():
    try:
        # Config browser options (headless mode)
        chrome_options = Options()
        chrome_options.add_argument("--headless")
        chrome_options.add_argument("--no-sandbox")
        chrome_options.add_argument("--disable-dev-shm-usage")
        
        # init  ChromeDriver
        service = Service(CHROMEDRIVER_PATH)
        driver = webdriver.Chrome(service=service, options=chrome_options)

        # Open DolarApp
        driver.get("https://www.dolarapp.com/es-AR")

        # Wait until it loads 
        wait = WebDriverWait(driver, 10)  # 10 sec wait
        rate_element = wait.until(
            EC.presence_of_element_located((By.CLASS_NAME, "flex.items-center.gap-3.text-xl.md\\:text-2xl.font-semibold.text-black"))
        )

        # Fetch item
        exchange_rate = rate_element.text

        # Close browser
        driver.quit()
        return exchange_rate
    except Exception as e:
        return f"[ERROR] {e}"

# Calls function and shows output
exchange_rate = fetch_exchange_rate()
print(f"Cotización actual: {exchange_rate}")
print(driver.page_source)

