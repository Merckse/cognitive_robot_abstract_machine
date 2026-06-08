import shutil

from customtkinter import *
from PIL import Image
from shutil import copy

filename = ""
def selectfile():
    global filename
    filename = filedialog.askopenfilename()
    filesplits = filename.split("/")
    if filesplits[len(filesplits)-1].split(".")[-1].lower() != "pdf":
        processing_text.configure(text="Please select a pdf file", font=("Monospace", 12), anchor="nw", text_color="darkred", padx=3, pady=3)

def copyfile():
    if filename != "":
        shutil.copy(filename, ".")
    else:
        print("No file selected")


app = CTk()

app.geometry("800x500")
app.resizable(False, False)

image_frame = CTkFrame(master=app, border_width=10, border_color="darkgrey")
main_frame = CTkFrame(master=app)

image = CTkImage(light_image=Image.open("penalty-score-generator.png"), size=(370,500))

side_image_label = CTkLabel(master=image_frame, image=image)

text = CTkLabel(master=main_frame, text="Base-Score-Penalty Generation", font=("Arial", 20))
pick_button = CTkButton(master=main_frame, text="Pick a file", corner_radius=5,command = selectfile)
gen_button = CTkButton(master=main_frame, text="Generate", corner_radius=5,command = copyfile)

processing_text = CTkLabel(master=main_frame, text="", font=("Arial", 20), bg_color="grey")


side_image_label.pack(anchor="w", expand=True,padx=10)

text.pack(expand=True, anchor="n", padx=5, pady=5)
processing_text.pack(expand=True, fill="both")
pick_button.pack(padx=5, pady=10)
gen_button.pack(padx=5, pady=10)

image_frame.pack(side="left", anchor="w",expand=True)
main_frame.pack(expand=True, side="right", fill="both")


app.mainloop()