import os
import shutil

from customtkinter import *
from PIL import Image
from pymupdf import pymupdf

filepath = ""
filename = ""
def selectfile():
    global filepath
    global filename
    filepath = filedialog.askopenfilename()
    filesplits = filepath.split("/")
    filename = filesplits[len(filesplits) - 1].lower()
    if filepath.split(".")[-1].lower() != "pdf":
        processing_text.configure(text="Please select a pdf file", font=("Monospace", 12), anchor="nw", text_color="darkred", padx=3, pady=3)
        return
    processing_text.configure(text=f"Picked file {filepath}")


"""

  The situation:                                                       
  - The PDF has no form fields — it's a flat/rendered document         
  - values.py has your BASE_POINTS, FLAT_PENALTIES, and                
  OUTCOME_MODIFIERS keyed by (ActionType, object_name)                 
  - The eval sheet (page 8, Pick & Place) lists actions with their     
  score values as plain text                                           
                                                                       
  Two broad approaches:                                                
                                                                       
  1. Overlay onto the existing PDF — use pymupdf's page.insert_text()  
  to write computed values at specific (x, y) coordinates into the     
  blank score columns ("1st try", "2nd try", "3rd try"). You'd need to 
  locate each row's coordinate, which you can do with                  
  page.search_for("Navigate to the table") to get the bounding box,    
  then offset right to the score column.                               
  2. Generate a new PDF from scratch — use a library like reportlab or 
  pymupdf's drawing API to programmatically build the entire sheet,    
  pulling all values directly from BASE_POINTS and FLAT_PENALTIES. More
  work upfront but fully data-driven.                                  
                                                                       
  The key challenge either way: mapping the action labels in the PDF   
  (e.g. "Picking up an object for transportation") to your (ActionType,
  object_name) keys. You'll need a mapping dict that bridges the two   
  namespaces.                                                          
                                                                       
  Recommended path: approach 1 — page.search_for(text) gives you the   
  row rect, then page.insert_text(rect.br + offset, str(score)) fills  
  the score column. Start with one page and one row to validate        
  coordinates, then generalize.  
  
"""
def generate_score_penalty_sheet():
    doc = pymupdf.open(filename)

    if os.path.exists("penalty-score-generator-temp.txt"):
        os.remove("penalty-score-generator-temp.txt")

    out = open("penalty-score-generator-temp.txt", "wb")

    for page in doc:
        text = page.get_text().encode("utf8")  # get plain text (is in UTF-8)
        out.write(text)  # write text of page
        out.write(bytes((12,)))  # write page delimiter (form feed 0x0C)
    out.close()

def copyfile():
    if filename != "":
        processing_text.configure(text=f"Copying file {filename}")
        if os.path.exists(filename):
            os.remove(filename)
        shutil.copy(filename, ".")
        generate_score_penalty_sheet()
    else:
        processing_text.configure(text=f"It appears that there has been no file selected.")
        print("No file selected")



app = CTk()

app.title("PDF Processor")
app.geometry("600x400")
app.resizable(False, False)

# --------------------frames
image_frame = CTkFrame(master=app)
main_frame = CTkFrame(master=app)
button_frame = CTkFrame(master=main_frame, fg_color="transparent")

# --------------------Elements
# ----Image_Elements
image = CTkImage(light_image=Image.open("penalty-score-generator.png"), size=(300,400))
side_image_label = CTkLabel(master=image_frame, image=image)

# ----Text_Elements
text = CTkLabel(master=main_frame, text="Base-Score-Penalty Generation", font=("Arial", 20))
processing_text = CTkLabel(master=main_frame, text="No file processed yet", font=("MonoSpace", 12), text_color="white")

# ----Buttons (the icons might lock AI-alike, but I just didn´t want to download icon packs...)
pick_button = CTkButton(master=button_frame, text="📂 Pick a file", corner_radius=5, command = selectfile)
gen_button = CTkButton(master=button_frame, text="⚙️ Generate", corner_radius=5 ,command = copyfile)


# --------------------Rendering
# -----Elements
text.pack( anchor="n", padx=5, pady=5)
side_image_label.pack(anchor="w", expand=True)

# -----Frames
image_frame.pack(side="left", anchor="w")
main_frame.pack(expand=True, side="right", fill="both")


button_frame.pack(anchor="w", fill="x")

pick_button.grid(row=0, column=0, padx=5, pady=5)
gen_button.grid(row=0, column=1, padx=5, pady=5)
processing_text.pack(anchor="w", padx=5, pady=15)

app.mainloop()