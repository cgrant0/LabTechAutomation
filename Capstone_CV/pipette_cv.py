import cv2
import numpy as np
import pytesseract
from pytesseract import Output

pytesseract.pytesseract.tesseract_cmd = 'C:/Program Files/Tesseract-OCR/tesseract.exe'

# Load the image
image_path = "pipette_0364.jpg"
image = cv2.imread(image_path)

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define the yellow color range to mask the yellow area
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([30, 255, 255])
yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

# Find contours in the yellow mask to get the yellow rectangle area
contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
roi = None
y_shift = 0
x_shift = 0

# Loop through contours to find the largest yellow area, which is likely the rectangle
for contour in contours:
    # Get bounding box for the yellow rectangle
    x, y, w, h = cv2.boundingRect(contour)
    # Draw a bounding box around the yellow rectangle on the original image
    if w * h >= 500:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        roi = image[y:y+h, x:x+w]
        y_shift = y
        x_shift = x

gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
img = cv2.bilateralFilter(gray, 9, 75, 75)
img = cv2.GaussianBlur(img,(5,5),0)
thresh = cv2.threshold(img, 70, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

newContours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
digits_img = []

x, y, w, h = cv2.boundingRect(newContours[0])
if w * h >= 500:
    cv2.rectangle(image, (x + x_shift, y + y_shift), (x + w + x_shift, y + h + y_shift), (0, 255, 0), 2)
    digits_img.append(thresh[y+10:y+h-10, x+10:x+w-10])

    for i in range(1,4):
        digit = thresh[y+10-h*i:y+h-h*i-20, x+10:x+w-10]
        padded_digit = cv2.copyMakeBorder(digit, 5, 5, 5, 5, cv2.BORDER_CONSTANT, value=(255, 255, 255))
        digits_img.append(padded_digit)
        cv2.rectangle(image, (x + x_shift, y + y_shift -h*i), (x + w + x_shift, y + h + y_shift - h*i), (0, 255, 0), 2)

custom_options = "--oem 3 --psm 6 -c tessedit_char_whitelist=0123456789"

digits = []

i = 0
for digit_img in digits_img:
    temp = pytesseract.image_to_boxes(digit_img, lang='eng', output_type=Output.DICT, config=custom_options)
    digits.insert(0, temp['char'][0])
    cv2.imshow(f"Digit {i}", digit_img)
    i += 1

print(f"Reading: {digits[0]}{digits[1]}{digits[2]}.{digits[3]} uL")
# Display the image with bounding boxes
cv2.imshow("Image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()