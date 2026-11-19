"""Dark palette for the mission controls and Matplotlib figures."""

BACKGROUND = '#10151e'
PANEL = '#171f2b'
BORDER = '#3b4b60'
TEXT = '#ecf3fb'
MUTED = '#b2c1d3'
ACCENT = '#65d9ef'
TRACE_COLORS = (ACCENT, '#ffbd78', '#b9a1ff')

DARK_PLOT_STYLE = {
    'figure.facecolor': BACKGROUND,
    'savefig.facecolor': BACKGROUND,
    'axes.facecolor': PANEL,
    'axes.edgecolor': BORDER,
    'axes.labelcolor': MUTED,
    'text.color': TEXT,
    'xtick.color': MUTED,
    'ytick.color': MUTED,
    'grid.color': BORDER,
    'legend.facecolor': PANEL,
    'legend.edgecolor': BORDER,
}


def apply_tk_dark_theme(root):
    from tkinter import ttk

    root.configure(background=BACKGROUND)
    style = ttk.Style(root)
    style.theme_use('clam')
    style.configure('.', background=BACKGROUND, foreground=TEXT,
                    bordercolor=BORDER, lightcolor=PANEL, darkcolor=PANEL,
                    troughcolor=PANEL, focuscolor=ACCENT)
    
    style.configure('TLabelframe.Label', background=BACKGROUND, foreground=MUTED)
    style.configure('TButton', background=PANEL, padding=(8, 4))
    style.map('TButton', background=[('pressed', BORDER), ('active', BORDER)],foreground=[('disabled', MUTED)])

    for control in ('TRadiobutton', 'TCheckbutton'):
        style.map(control, background=[('active', PANEL)], foreground=[('disabled', MUTED)], indicatorcolor=[('selected', ACCENT), ('!selected', PANEL)])

    style.configure('Horizontal.TScale', background=BORDER, troughcolor=PANEL)
    style.map('Horizontal.TScale', background=[('active', ACCENT)])


def save_window_image(root, path):
    """Save the final Tk window as a PNG before closing it."""
    from PIL import ImageGrab

    try:
        root.deiconify()
        windows = root.tk.call('tk', 'windowingsystem') == 'win32'
        if not windows:
            # Bring the client area in front for the screen crop.
            root.lift()
        # Paint restored windows before grabbing the pixels.
        root.update()
        if windows:
            image = ImageGrab.grab(window=int(root.frame(), 0))
        else:
            left, top = root.winfo_rootx(), root.winfo_rooty()
            image = ImageGrab.grab(bbox=(left, top, left + root.winfo_width(),
                                         top + root.winfo_height()))
        image.save(path)
        print(f'Window image saved: {path}', flush=True)
    except OSError as error:
        print(f'Could not save window image {path}: {error}', flush=True)
