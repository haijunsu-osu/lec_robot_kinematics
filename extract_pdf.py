import PyPDF2
import sys

pdf_path = r"c:\Users\haiju\OneDrive\Documents\github_repo\teaching\robot_kinematics\Chapter_12 McCarthy.pdf"

try:
    with open(pdf_path, 'rb') as file:
        pdf_reader = PyPDF2.PdfReader(file)
        print(f"Total pages: {len(pdf_reader.pages)}\n")
        
        # Extract text from all pages
        for i, page in enumerate(pdf_reader.pages):
            text = page.extract_text()
            print(f"=== Page {i+1} ===")
            print(text)
            print("\n" + "="*80 + "\n")
            
except Exception as e:
    print(f"Error: {e}")
    print("Trying pdfplumber...")
    
    try:
        import pdfplumber
        with pdfplumber.open(pdf_path) as pdf:
            print(f"Total pages: {len(pdf.pages)}\n")
            for i, page in enumerate(pdf.pages):
                text = page.extract_text()
                print(f"=== Page {i+1} ===")
                print(text)
                print("\n" + "="*80 + "\n")
    except Exception as e2:
        print(f"Error with pdfplumber: {e2}")
