import os
import json
from dotenv import load_dotenv
from twilio.rest import Client
from apscheduler.schedulers.background import BackgroundScheduler
from datetime import datetime
import google.generativeai as genai

# ==== Configuración ====
load_dotenv()
scheduler = BackgroundScheduler()
scheduler.start()

# Configura Gemini con tu API key
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
model = genai.GenerativeModel('gemini-1.5-pro')

# Twilio
twilio_client = Client(os.getenv("TWILIO_SID"), os.getenv("TWILIO_TOKEN"))

# ==== WhatsApp ====
def send_whatsapp(message="Recordatorio: "):
    """
    Función para enviar un mensaje de WhatsApp a través de Twilio.
    """
    twilio_client.messages.create(
        body=message,
        from_=os.getenv("TWILIO_NUMBER"),
        to=os.getenv("MY_NUMBER")
    )
    print(f"✅ WhatsApp enviado: {message}")

# ==== Usar AI para responder preguntas ====
def ask_ai(prompt: str) -> str:
    """
    Función para enviar un prompt a Gemini y obtener una respuesta de texto.
    """
    try:
        response = model.generate_content(prompt)
        return response.text
    except Exception as e:
        return f"Error al generar la respuesta: {e}"

# ==== Parseo de recordatorios con Gemini ====
def parse_reminder_with_gemini(frase: str) -> dict:
    """
    Usa Gemini para transformar una frase en JSON con fecha, hora y mensaje.
    """
    prompt = f"""
    Convierte la siguiente frase en un JSON con el formato:
    {{
      "fecha": "YYYY-MM-DD",
      "hora": "HH:MM",
      "mensaje": "..."
    }}
    Asegúrate de que la fecha sea una fecha futura y de que el JSON sea válido.
    Frase: "{frase}"
    """
    response = model.generate_content(prompt)
    try:
        # Gemini puede generar texto adicional, así que extraemos solo el JSON
        json_str = response.text.strip().replace('```json\n', '').replace('\n```', '')
        return json.loads(json_str)
    except json.JSONDecodeError as e:
        print(f"Error al decodificar JSON: {e}")
        print(f"Respuesta de Gemini: {response.text}")
        return None

# ==== Programar recordatorio ====
def set_reminder(frase: str):
    reminder = parse_reminder_with_gemini(frase)
    if not reminder:
        print("❌ No se pudo programar el recordatorio. Formato inválido.")
        return

    reminder_datetime = datetime.strptime(
        f"{reminder['fecha']} {reminder['hora']}", "%Y-%m-%d %H:%M"
    )

    scheduler.add_job(
        send_whatsapp,
        "date",
        run_date=reminder_datetime,
        args=[reminder["mensaje"]],
    )
    print("📅 Recordatorio agendado:", reminder)

# ==== Configuración de la aplicación con Gemini ====
def setup_app_with_gemini(user_input: str):
    print("🤖 Adaptando la aplicación con Gemini...")
    prompt = f"""
    Un usuario quiere que la aplicación se adapte a sus preferencias basándose en esta frase: "{user_input}".
    Genera un JSON con las siguientes preferencias extraídas del texto del usuario:
    - "estilo": un valor como 'texto', 'audio' o 'colores'.
    - "ui": un valor como 'minimalista' o 'colorida'.
    Si la preferencia no se menciona, usa el valor 'predeterminado'.
    """
    response = model.generate_content(prompt)
    try:
        json_str = response.text.strip().replace('```json\n', '').replace('\n```', '')
        preferencias = json.loads(json_str)
        with open("preferencias.json", "w") as f:
            json.dump(preferencias, f)
        print("✅ Preferencias guardadas:", preferencias)
    except json.JSONDecodeError as e:
        print(f"Error al decodificar JSON de preferencias: {e}")
