import csv
import random
import sys
N = int(sys.argv[1]) if len(sys.argv) > 1 else 20000
SEED = int(sys.argv[2]) if len(sys.argv) > 2 else 1
random.seed(SEED)
HI = ["blood bags", "blood units", "plasma", "oxygen cylinder", "oxygen mask", "defibrillator pads", "insulin", "epinephrine", "antivenom", "donor organ", "platelets", "crash cart supplies", "ventilator tubes", "adrenaline", "nitroglycerin", "emergency medicines", "trauma kit", "cardiac drugs"]
MID = ["antibiotics", "surgical tools", "lab samples", "iv fluids", "bandages", "painkillers", "x-ray films", "sterile gloves", "medicines", "syringes", "suture kit", "saline bags", "test reports", "blood test samples", "dressing kit", "catheters", "face masks", "scalpels", "vaccines", "prescription drugs"]
LO = ["lunch trays", "clean linen", "towels", "stationery", "cleaning supplies", "magazines", "patient meals", "pillows", "blankets", "files", "bed sheets", "tissue boxes", "water bottles", "paper rolls", "newspapers", "uniforms", "toiletries", "breakfast trays", "folders", "printer paper"]
UP = ["stat", "urgent", "emergency", "critical", "asap", "life saving", "immediately", "code blue"]
DN = ["routine", "non urgent", "when free", "no rush", "later today", "low priority"]
DEPTS = ["Ward", "ICU", "Surgery", "Pharmacy"]
PRE = ["", "", "", "", "please deliver ", "send ", "bring ", "need ", "deliver ", "take ", "carry ", "fetch "]
SUF = ["", "", "", "", " now", " today", " to the room", " for patient", " please"]
def typo(t):
    if len(t) < 5 or random.random() > 0.12:
        return t
    i = random.randint(1, len(t) - 2)
    k = random.random()
    if k < 0.4:
        return t[:i] + t[i + 1:]
    if k < 0.7:
        return t[:i] + t[i + 1] + t[i] + t[i + 2:]
    return t[:i] + t[i] + t[i:]
rows = []
for _ in range(N):
    g = random.choices((0, 1, 2), weights=(3, 3, 4))[0]
    it = random.choice((HI, MID, LO)[g])
    b = 3 - g
    t = it
    m = random.random()
    if m < 0.15:
        t = random.choice(UP) + " " + it
        b = 3
    elif m < 0.30:
        t = it + " " + random.choice(DN)
        b = max(1, b - 1)
    t = typo(random.choice(PRE) + t + random.choice(SUF)).strip()
    if random.random() < 0.2:
        t = t.upper() if random.random() < 0.5 else t.capitalize()
    s = random.choice(DEPTS)
    h = random.randint(0, 23)
    n = h < 6 or h >= 22
    if b == 2 and (s == "ICU" or n):
        b = 3
    y = b
    if random.random() < 0.05:
        y = random.randint(1, 3)
    rows.append((t, s, h, y))
with open("dataset.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["d", "s", "h", "y"])
    w.writerows(rows)
c = {1: 0, 2: 0, 3: 0}
for r in rows:
    c[r[3]] += 1
print(f"Saved dataset.csv with {len(rows)} rows (seed {SEED})")
print("Class counts:", c)