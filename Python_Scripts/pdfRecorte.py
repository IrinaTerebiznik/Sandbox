# This script extracts specific pages from a PDF file (doc.pdf) and saves them into a new PDF file (output.pdf).
# Note that this script must be saved in the same folder as the pdf file to change

from PyPDF2 import PdfReader, PdfWriter

reader = PdfReader("doc.pdf")  # Change with the file you want
writer = PdfWriter()
# Pages to keep (it starts in 0)
pages_to_keep = [0,1, 2, 3]  
for page_num in pages_to_keep:
    writer.add_page(reader.pages[page_num])

with open("output.pdf", "wb") as output_file: # Change "output.pdf" with the desire name for the new file
    writer.write(output_file)
