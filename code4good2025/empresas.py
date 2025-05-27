# [IT] TO DO: Configurr otras opciones y el return to menu
import logging
from dotenv import load_dotenv
import os
import pandas as pd
from telegram import Update
from telegram.ext import (
    ApplicationBuilder, CommandHandler, MessageHandler, ConversationHandler,
    ContextTypes, filters
)
import time
from datetime import date
import asyncio

# Configuración de logging
logging.basicConfig(
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    level=logging.INFO
)

# Estados de la conversación
MENU, EMPRESA, FECHA, PRODUCTO, CANTIDAD, CONFIRMAR_OTRO = range(6)

EXCEL_PATH = "pedidos.xlsx"

# Crear archivo si no existe
def inicializar_excel():
    if not os.path.exists(EXCEL_PATH):
        pedidos = pd.DataFrame(columns=["empresa", "fecha de entrega", "producto", "cantidad", "fecha de realización"])
        pedidos.to_excel(EXCEL_PATH, index=False)

# Funciones del bot
async def start(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    await update.message.reply_text(
        "Hola 👋 ¿Qué querés hacer? \n1️⃣ Realizar un pedido \n2️⃣ Conseguir números de contacto \n3️⃣ Ofrecerte como voluntario \n4️⃣ Trabajar para la empresa \n5️⃣ Dejar una sugerencia\n6️⃣ Hablar con alguien"
    )
    return MENU

async def menu(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    opcion = update.message.text.strip()
    if opcion == "1":
        await update.message.reply_text("📝 Ingresá el nombre de tu empresa:")
        return EMPRESA
    elif opcion == "2":
        await update.message.reply_text("📞 Contacto: Carla 11445453442")
        return await start(update, context)
    elif opcion == "3":
        await update.message.reply_text("🙏 ¡Gracias por tu interés! Envíanos tu propuesta por este medio.")
        return await start(update, context)
    elif opcion == "4":
        await update.message.reply_text("💼 Podés mandarnos tu CV o mensaje por este medio.")
        return await start(update, context)
    elif opcion == "5":
        await update.message.reply_text("✉️ Escribinos tu sugerencia por este chat.")
        return await start(update, context)
    elif opcion == "6":
        await update.message.reply_text("🙋 Te pondremos en contacto con alguien de nuestro equipo pronto.")
        await asyncio.sleep(5)
        await update.message.reply_text("❌ No hay nadie disponible en este momento ")
        return await start(update, context)
    else:
        await update.message.reply_text("❌ Opción no válida. Escribí un número del 1 al 6.")
        return await start(update, context)

# Pedido paso a paso
async def recibir_empresa(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    texto = update.message.text.strip()
    if texto == "0":
        return await start(update, context)
    context.user_data["empresa"] = texto
    await update.message.reply_text("📅 ¿Para qué fecha es el pedido? (formato YYYY-MM-DD)\n Ingresa 0️⃣ si queres volver al menu principal")
    return FECHA

async def recibir_fecha(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    texto = update.message.text.strip()
    if texto == "0":
        return await start(update, context)
    context.user_data["fecha"] = texto
    context.user_data["items"] = []
    await update.message.reply_text("📦 ¿Qué producto querés pedir?\n Ingresa 0️⃣ si queres volver al menu principal y cancelar la operacion")
    return PRODUCTO

async def recibir_producto(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    texto = update.message.text.strip()
    if texto == "0":
        return await start(update, context)
    context.user_data["producto"] = texto
    await update.message.reply_text("🔢 ¿Qué cantidad?\n Ingresa 0️⃣ si queres volver al menu principal y cancelar la operacion")
    return CANTIDAD

async def recibir_cantidad(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    texto = update.message.text.strip()
    if texto == "0":
        return await start(update, context)
    producto = context.user_data["producto"]
    context.user_data["items"].append({
        "producto": producto,
        "cantidad": texto
    })
    await update.message.reply_text("¿Querés agregar otro producto? (sí / no)\n Ingresa 0️⃣ si queres volver al menu principal y cancelar la operacion")
    return CONFIRMAR_OTRO

async def confirmar_otro(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    texto = update.message.text.strip().lower()
    if texto == "0":
        return await start(update, context)
    if texto in ["sí", "si"]:
        await update.message.reply_text("📦 ¿Qué otro producto querés pedir?\n Ingresa 0️⃣ si queres volver al menu principal y cancelar la operacion")
        return PRODUCTO
    else:
        empresa = context.user_data["empresa"]
        fecha_entrega = context.user_data["fecha"]
        fecha_realizacion = date.today().isoformat()
        items = context.user_data["items"]
        df = pd.read_excel(EXCEL_PATH)
        for item in items:
            df = pd.concat([df, pd.DataFrame([{
                "empresa": empresa,
                "fecha de entrega": fecha_entrega,
                "producto": item["producto"],
                "cantidad": item["cantidad"],
                "fecha de realización": fecha_realizacion
            }])], ignore_index=True)
        df.to_excel(EXCEL_PATH, index=False)
        await update.message.reply_text("✅ Pedido guardado correctamente. ¡Gracias!")
        return await start(update, context)

async def cancelar(update: Update, context: ContextTypes.DEFAULT_TYPE) -> int:
    await update.message.reply_text("🚫 Operación cancelada.")
    return ConversationHandler.END

# Construcción del bot
def build_app(token: str):
    app = ApplicationBuilder().token(token).build()
    conv_handler = ConversationHandler(
        entry_points=[CommandHandler("start", start), MessageHandler(filters.TEXT & ~filters.COMMAND, start)],
        states={
            MENU: [MessageHandler(filters.TEXT & ~filters.COMMAND, menu)],
            EMPRESA: [MessageHandler(filters.TEXT & ~filters.COMMAND, recibir_empresa)],
            FECHA: [MessageHandler(filters.TEXT & ~filters.COMMAND, recibir_fecha)],
            PRODUCTO: [MessageHandler(filters.TEXT & ~filters.COMMAND, recibir_producto)],
            CANTIDAD: [MessageHandler(filters.TEXT & ~filters.COMMAND, recibir_cantidad)],
            CONFIRMAR_OTRO: [MessageHandler(filters.TEXT & ~filters.COMMAND, confirmar_otro)],
        },
        fallbacks=[CommandHandler("cancelar", cancelar)],
    )
    app.add_handler(conv_handler)
    return app

if __name__ == "__main__":
    inicializar_excel()
    load_dotenv(dotenv_path="venv/config/.env")  
    TOKEN = os.getenv("EMPRESAS_TOKEN_BOT")
    asyncio.run(build_app(TOKEN).run_polling())
