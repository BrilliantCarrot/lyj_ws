from PIL import Image
import pytesseract

# Load the image to extract text
image_path = "C:/Users/leeyj/OneDrive/사진/코믹스 번역/Main Series 52권/03.jpg"
img = Image.open(image_path)

# Use OCR to extract text from the image
extracted_text = pytesseract.image_to_string(img)

extracted_text