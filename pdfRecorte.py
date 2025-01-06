from PyPDF2 import PdfReader, PdfWriter

reader = PdfReader("doc.pdf")
writer = PdfWriter()

pages_to_keep = [0,1, 2, 3]  # Índices de las páginas que deseas mantener (comienza en 0)
for page_num in pages_to_keep:
    writer.add_page(reader.pages[page_num])

with open("output.pdf", "wb") as output_file:
    writer.write(output_file)
