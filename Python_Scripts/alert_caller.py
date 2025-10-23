from flask import Flask, request
from twilio.rest import Client
import requests

app = Flask(__name__)

# Twilio config
TWILIO_SID = "TU_ACCOUNT_SID"
TWILIO_TOKEN = "TU_AUTH_TOKEN"
TWILIO_NUMBER = "+1xxxxxxxxxx"  

# Freshdesk config
FRESHDESK_API_KEY = "tu_api_key"
FRESHDESK_DOMAIN = "tudominio.freshdesk.com"

client = Client(TWILIO_SID, TWILIO_TOKEN)

def get_on_call_agent_phone():
    # Ejemplo simple: obtenemos los agentes del grupo "roberto"
    url = f"https://{FRESHDESK_DOMAIN}/api/v2/groups"
    response = requests.get(url, auth=(FRESHDESK_API_KEY, "X"))
    groups = response.json()

    group_id = next((g["id"] for g in groups if g["name"].lower() == "roberto"), None)
    if not group_id:
        return None

    # Obtener miembros del grupo
    url = f"https://{FRESHDESK_DOMAIN}/api/v2/group_memberships"
    response = requests.get(url, auth=(FRESHDESK_API_KEY, "X"))
    memberships = response.json()

    agent_ids = [m["user_id"] for m in memberships if m["group_id"] == group_id]

    # Buscar un agente disponible
    for agent_id in agent_ids:
        url = f"https://{FRESHDESK_DOMAIN}/api/v2/agents/{agent_id}"
        agent = requests.get(url, auth=(FRESHDESK_API_KEY, "X")).json()
        if agent.get("available", True):  # Simplificado: se puede mejorar
            return agent.get("mobile") or agent.get("phone")

    return None

@app.route("/webhook", methods=["POST"])
def webhook():
    data = request.json
    subject = data.get("ticket", {}).get("subject", "Sin asunto")

    phone = get_on_call_agent_phone()
    if not phone:
        return {"status": "No on-call agent found"}, 404

    call = client.calls.create(
        to=phone,
        from_=TWILIO_NUMBER,
        twiml=f'<Response><Say voice="alice">Se creó un nuevo ticket: {subject}</Say></Response>'
    )

    return {"status": "llamada iniciada", "sid": call.sid}, 200

if __name__ == "__main__":
    app.run(port=5000)
