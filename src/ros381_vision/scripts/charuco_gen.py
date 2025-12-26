import cv2
import numpy as np

# ===================== BOARD PARAMETERS =====================
SQUARES_X = 8
SQUARES_Y = 11

SQUARE_LENGTH_MM = 20   # chessboard square size (mm)
MARKER_LENGTH_MM = 15   # marker size (mm)

DICT_ID = cv2.aruco.DICT_4X4_50

# ===================== PRINT SETTINGS ======================
DPI = 300
PAPER_W_MM = 210        # A4 width
PAPER_H_MM = 297        # A4 height
MARGIN_MM = 15          # REQUIRED margin
# ===========================================================

def mm_to_px(mm):
    return int(mm * DPI / 25.4)

# Convert sizes
square_px = mm_to_px(SQUARE_LENGTH_MM)
marker_px = mm_to_px(MARKER_LENGTH_MM)

margin_px = mm_to_px(MARGIN_MM)
paper_w_px = mm_to_px(PAPER_W_MM)
paper_h_px = mm_to_px(PAPER_H_MM)

board_w_px = SQUARES_X * square_px
board_h_px = SQUARES_Y * square_px

# Sanity check
if board_w_px + 2 * margin_px > paper_w_px or board_h_px + 2 * margin_px > paper_h_px:
    raise RuntimeError("Board + margins do NOT fit on A4. Reduce square size.")

# Create board
aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)
board = cv2.aruco.CharucoBoard(
    (SQUARES_X, SQUARES_Y),
    SQUARE_LENGTH_MM / 1000.0,   # meters
    MARKER_LENGTH_MM / 1000.0,   # meters
    aruco_dict
)

# Generate board image (just the board)
board_img = board.generateImage(
    (board_w_px, board_h_px),
    marginSize=0,
    borderBits=1
)

# Create full white A4 canvas
canvas = np.full((paper_h_px, paper_w_px), 255, dtype=np.uint8)

# Center board with margins
x0 = (paper_w_px - board_w_px) // 2
y0 = (paper_h_px - board_h_px) // 2

# Enforce minimum margin
if x0 < margin_px or y0 < margin_px:
    raise RuntimeError("Margins ended up smaller than requested.")

canvas[y0:y0 + board_h_px, x0:x0 + board_w_px] = board_img

# Save
cv2.imwrite("charuco_8x11_A4_margin15mm.png", canvas)

print("Saved: charuco_8x11_A4_margin15mm.png")
print("PRINT AT 100% SCALE (no fit-to-page)")