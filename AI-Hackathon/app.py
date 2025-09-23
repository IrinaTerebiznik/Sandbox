import streamlit as st
from backend import send_whatsapp, ask_ai

st.title("MVP Estudio")

# Botón para enviar recordatorio
if st.button("Enviar recordatorio"):
    send_whatsapp()
    st.success("¡Recordatorio enviado!")

# Preguntar a la AI
question = st.text_input("Escribí tu pregunta sobre estudio")
if st.button("Preguntar AI"):
    if question.strip():
        answer = ask_ai(question)
        st.write(answer)
    else:
        st.warning("Ingresá una pregunta primero")
