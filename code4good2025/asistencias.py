import logging
from dotenv import load_dotenv

logging.basicConfig(
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    level=logging.INFO
)

from telegram import Update
from telegram.ext import (
    ApplicationBuilder, CommandHandler, MessageHandler, filters,
    ConversationHandler, ContextTypes
)
import pandas as pd
from datetime import date
import os
from telegram.ext import ConversationHandler, CallbackContext

USUARIO, CONTRASENA, MENU, ASISTENCIA, ENTRADA, SALIDA, JUSTIFICACION, CERTIFICADO, CANCELAR = range(9)

EXCEL_PATH = "trabajadores.xlsx"
CERT_FOLDER = "certificados"

def inicializar_excel():
    if not os.path.exists(EXCEL_PATH):
        usuarios = pd.DataFrame([{"usuario": "marcos", "contrasena": "1234", "nombre": "Marcos"}])
        asistencias = pd.DataFrame(columns=["usuario", "fecha", "asistencia", "entrada", "salida", "justificacion", "certificado"])
        with pd.ExcelWriter(EXCEL_PATH) as writer:
            usuarios.to_excel(writer, sheet_name="usuarios", index=False)
            asistencias.to_excel(writer, sheet_name="asistencias", index=False)
    os.makedirs(CERT_FOLDER, exist_ok=True)

async def start(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    await update.message.reply_text("👋 Bienvenido. Ingresá tu usuario:")
    return USUARIO

async def recibir_usuario(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    context.user_data["usuario"] = update.message.text.strip()
    return await mostrar_menu(update, context)

async def mostrar_menu(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    await update.message.reply_text(f"Bienvenido {context.user_data['usuario']}\n¿Qué querés hacer?\n 1️⃣ Registrar asistencia del día\n 2️⃣ Ver mis faltas")
    return MENU

async def menu(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    opcion = update.message.text.strip()
    if opcion == "1":
        await update.message.reply_text("📅 ¿Asististe hoy? (sí / no)\n Ingresa 0️⃣ si queres volver al menú principal")
        return ASISTENCIA
    elif opcion == "2":
        await update.message.reply_text("📊 Tenés 3 faltas registradas este mes.")
        return await mostrar_menu(update, context)
    else:
        await update.message.reply_text("❗ Opción inválida. Escribí 1 para registrar asistencia, o 2 para ver tus faltas.")
        return await mostrar_menu(update, context)

async def recibir_asistencia(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    respuesta = update.message.text.lower().strip()
    if respuesta == "0":
        return await mostrar_menu(update, context)
    context.user_data["fecha"] = date.today().isoformat()
    usuario = context.user_data["usuario"]
    df = pd.read_excel(EXCEL_PATH, sheet_name="asistencias")
    mask = (df["usuario"] == usuario) & (df["fecha"] == context.user_data["fecha"])
    if respuesta in ["sí", "si"]:
        if df[mask].empty or pd.isna(df.loc[mask].iloc[0]["entrada"]):
            await update.message.reply_text("🕗 ¿A qué hora ingresaste? (formato HH:MM)\n Ingresa 0️⃣ si queres volver al menú principal")
            return ENTRADA
        else:
            await update.message.reply_text("🕔 ¿A qué hora saliste? (formato HH:MM)\n Ingresa 0️⃣ si queres volver al menú principal")
            return SALIDA
    elif respuesta == "no":
        await update.message.reply_text("✏️ Escribí una breve justificación:\n Ingresa 0️⃣ si queres volver al menú principal")
        return JUSTIFICACION
    else:
        await update.message.reply_text("❗ Escribí 'Sí' o 'No'.")
        return ASISTENCIA

async def recibir_entrada(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    texto = update.message.text.strip()
    if texto == "0":
        return await mostrar_menu(update, context)
    context.user_data["entrada"] = texto
    return await guardar_asistencia(update, context, asistencia="presente")

async def recibir_salida(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    texto = update.message.text.strip()
    if texto == "0":
        return await mostrar_menu(update, context)
    context.user_data["salida"] = texto
    return await guardar_asistencia(update, context)

async def recibir_justificacion(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    texto = update.message.text.strip()
    if texto == "0":
        return await mostrar_menu(update, context)
    context.user_data["justificacion"] = texto
    await update.message.reply_text("📎 Enviá un archivo (PDF o DOC) como certificado.")
    return CERTIFICADO

async def recibir_certificado(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    doc = update.message.document
    if not doc:
        await update.message.reply_text("❌ No se detectó un archivo. Volviendo al menú principal.")
        return await mostrar_menu(update, context)
    
    mimetype = doc.mime_type
    extensiones_validas = [
        "application/pdf",
        "application/msword",
        "application/vnd.openxmlformats-officedocument.wordprocessingml.document"
    ]
    if mimetype not in extensiones_validas:
        await update.message.reply_text("❌ El archivo no es un PDF ni un DOC válido. Volviendo al menú principal.")
        return await mostrar_menu(update, context)

    file_path = os.path.join(CERT_FOLDER, doc.file_name)
    await doc.get_file().download_to_drive(file_path)
    context.user_data["certificado"] = doc.file_name
    return await guardar_asistencia(update, context, asistencia="ausente")

async def guardar_asistencia(update: Update, context: ContextTypes.DEFAULT_TYPE, asistencia=None) -> int:
    usuario = context.user_data["usuario"]
    fecha = context.user_data["fecha"]
    entrada = context.user_data.get("entrada", "")
    salida = context.user_data.get("salida", "")
    justificacion = context.user_data.get("justificacion", "")
    certificado = context.user_data.get("certificado", "")
    df = pd.read_excel(EXCEL_PATH, sheet_name="asistencias")
    mask = (df["usuario"] == usuario) & (df["fecha"] == fecha)
    if not df[mask].empty:
        idx = df[mask].index[0]
        if entrada: df.at[idx, "entrada"] = entrada
        if salida: df.at[idx, "salida"] = salida
        if justificacion: df.at[idx, "justificacion"] = justificacion
        if certificado: df.at[idx, "certificado"] = certificado
    else:
        nueva = {
            "usuario": usuario, "fecha": fecha, "asistencia": asistencia,
            "entrada": entrada, "salida": salida,
            "justificacion": justificacion, "certificado": certificado
        }
        df = pd.concat([df, pd.DataFrame([nueva])], ignore_index=True)
    with pd.ExcelWriter(EXCEL_PATH, mode="a", if_sheet_exists="replace", engine="openpyxl") as writer:
        df.to_excel(writer, sheet_name="asistencias", index=False)
    await update.message.reply_text("✅ Registro guardado correctamente. ¡Gracias!")
    return await mostrar_menu(update, context)

async def cancelar(update: Update, context: ContextTypes.DEFAULT_TYPE):
    await update.message.reply_text("🚫 Operación cancelada.")
    return ConversationHandler.END

def build_app(token: str):
    app = ApplicationBuilder().token(token).build()
    conv_handler = ConversationHandler(
        entry_points=[CommandHandler("start", start)],
        states={
            USUARIO: [MessageHandler(filters.TEXT & ~filters.COMMAND, recibir_usuario)],
            MENU: [MessageHandler(filters.TEXT & ~filters.COMMAND, menu)],
            ASISTENCIA: [MessageHandler(filters.TEXT & ~filters.COMMAND, recibir_asistencia)],
            ENTRADA: [MessageHandler(filters.TEXT & ~filters.COMMAND, recibir_entrada)],
            SALIDA: [MessageHandler(filters.TEXT & ~filters.COMMAND, recibir_salida)],
            JUSTIFICACION: [MessageHandler(filters.TEXT & ~filters.COMMAND, recibir_justificacion)],
            CERTIFICADO: [MessageHandler(filters.Document.ALL, recibir_certificado)],
        },
        fallbacks=[CommandHandler("cancelar", cancelar)],
    )
    app.add_handler(conv_handler)
    return app

# Ejecutar el bot
if __name__ == "__main__":
    import asyncio
    inicializar_excel()
    load_dotenv(dotenv_path="venv/config/.env")
    TOKEN = os.getenv("ASISTENCIA_TOKEN_BOT")
    asyncio.run(build_app(TOKEN).run_polling())
