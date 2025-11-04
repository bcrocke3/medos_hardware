import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog
import numpy as np

class BinaryGridApp:
    def __init__(self, root, rows=20, cols=20, cell_size=5):
        self.root = root
        self.root.title("Binary Map Editor (NumPy)")
        self.cell_size = cell_size

        self.rows = rows
        self.cols = cols
        self.grid = np.zeros((rows, cols), dtype=int)
        self.rects = {}

        # Canvas
        self.canvas = tk.Canvas(root, bg="white", highlightthickness=0)
        self.canvas.pack(padx=10, pady=10)

        # Buttons
        btn_frame = tk.Frame(root)
        btn_frame.pack(pady=5)
        tk.Button(btn_frame, text="Save as .npy", command=self.save_array).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Load .npy", command=self.load_array).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Print to Console", command=self.print_array).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Clear", command=self.clear_grid).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Resize Grid", command=self.resize_grid_dialog).pack(side="left", padx=5)

        # Build grid
        self.build_grid()
        self.canvas.bind("<Button-1>", self.toggle_cell)

    def build_grid(self):
        """Draws the grid rectangles based on current size."""
        self.canvas.delete("all")
        self.rects.clear()

        width = self.cols * self.cell_size
        height = self.rows * self.cell_size
        self.canvas.config(width=width, height=height)

        for r in range(self.rows):
            for c in range(self.cols):
                x1, y1 = c * self.cell_size, r * self.cell_size
                x2, y2 = x1 + self.cell_size, y1 + self.cell_size
                color = "black" if self.grid[r, c] else "white"
                rect = self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="gray")
                self.rects[(r, c)] = rect

        # Force window resize to fit content
        self.root.update_idletasks()
        self.root.geometry(f"{width + 40}x{height + 150}")

    def toggle_cell(self, event):
        """Toggle cell on click."""
        c = event.x // self.cell_size
        r = event.y // self.cell_size
        if 0 <= r < self.rows and 0 <= c < self.cols:
            self.grid[r, c] = 1 - self.grid[r, c]
            color = "black" if self.grid[r, c] else "white"
            self.canvas.itemconfig(self.rects[(r, c)], fill=color)

    def save_array(self):
        """Save the current grid as a NumPy .npy file."""
        path = filedialog.asksaveasfilename(
            defaultextension=".npy",
            filetypes=[("NumPy array", "*.npy")],
            title="Save grid as NumPy array"
        )
        if path:
            np.save(path, self.grid)
            messagebox.showinfo("Saved", f"Grid saved as NumPy array at:\n{path}")

    def load_array(self):
        """Load a NumPy array into the grid."""
        path = filedialog.askopenfilename(
            defaultextension=".npy",
            filetypes=[("NumPy array", "*.npy")],
            title="Load grid from NumPy array"
        )
        if path:
            try:
                data = np.load(path)
                self.rows, self.cols = data.shape
                self.grid = data
                self.build_grid()
                messagebox.showinfo("Loaded", f"Loaded grid from {path}")
            except Exception as e:
                messagebox.showerror("Error", f"Could not load file:\n{e}")

    def print_array(self):
        print(self.grid)

    def clear_grid(self):
        self.grid.fill(0)
        self.build_grid()

    def resize_grid_dialog(self):
        """Prompt user for new grid size."""
        new_rows = simpledialog.askinteger("Resize Grid", "Enter number of rows:", initialvalue=self.rows, minvalue=1)
        if new_rows is None:
            return
        new_cols = simpledialog.askinteger("Resize Grid", "Enter number of columns:", initialvalue=self.cols, minvalue=1)
        if new_cols is None:
            return
        self.resize_grid(new_rows, new_cols)

    def resize_grid(self, new_rows, new_cols):
        """Resize grid, preserving data if possible."""
        new_grid = np.zeros((new_rows, new_cols), dtype=int)
        min_rows = min(self.rows, new_rows)
        min_cols = min(self.cols, new_cols)
        new_grid[:min_rows, :min_cols] = self.grid[:min_rows, :min_cols]

        self.rows, self.cols = new_rows, new_cols
        self.grid = new_grid
        self.build_grid()

if __name__ == "__main__":
    root = tk.Tk()
    app = BinaryGridApp(root, rows=20, cols=20, cell_size=15)
    root.mainloop()
