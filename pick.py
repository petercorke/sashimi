from machinevisiontoolbox import *
import pickle

knife = Image.Read(
    "/Users/corkep/Dropbox/code/knife_path/Global-G-14-Sashimimes-Spits-30cm-800x800.webp"
)
knife.disp()

pts = plt.ginput(-1, timeout=0)
print(pts)

with open("perim.p", "wb") as f:
    pickle.dump(pts, f)
