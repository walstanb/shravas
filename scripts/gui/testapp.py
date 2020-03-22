import Tkinter as tk
import ttk as ttk

root = tk.Tk()
frame = tk.Frame(root)
frame.grid()
s = ttk.Style()
s.theme_use('clam')
TROUGH_COLOR = 'blue'
BAR_COLOR = 'green'
s.configure("Horizontal.TProgressbar", troughcolor=TROUGH_COLOR, bordercolor=TROUGH_COLOR, background=BAR_COLOR, lightcolor=BAR_COLOR, darkcolor=BAR_COLOR)
ttk.Progressbar(frame, style="Horizontal.TProgressbar", orient="horizontal", length=600, mode="determinate", maximum=4, value=1).grid(row=1, column=1)
frame.pack()

root.mainloop()