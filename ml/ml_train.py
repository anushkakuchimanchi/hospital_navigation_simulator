import numpy as np
import pandas as pd
import joblib
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.compose import ColumnTransformer
from sklearn.pipeline import Pipeline
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.preprocessing import OneHotEncoder
from sklearn.linear_model import LogisticRegression
from sklearn.svm import LinearSVC
from sklearn.naive_bayes import MultinomialNB
from sklearn.ensemble import RandomForestClassifier
from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import accuracy_score, confusion_matrix, ConfusionMatrixDisplay
try:
    df = pd.read_csv("dataset.csv")
except FileNotFoundError:
    raise SystemExit("dataset.csv not found. Run: python gen_dataset.py")
df["d"] = df["d"].fillna("").astype(str).str.lower()
df["n"] = ((df["h"] < 6) | (df["h"] >= 22)).astype(int)
print(f"Loaded {len(df)} rows")
X = df[["d", "s", "n"]]
y = df["y"]
def pre():
    return ColumnTransformer([
        ("t", TfidfVectorizer(analyzer="char_wb", ngram_range=(2, 4)), "d"),
        ("c", OneHotEncoder(handle_unknown="ignore"), ["s"]),
        ("n", "passthrough", ["n"])])
models = {
    "Logistic Regression": LogisticRegression(max_iter=1000),
    "Linear SVM": LinearSVC(),
    "Naive Bayes": MultinomialNB(),
    "Random Forest": RandomForestClassifier(n_estimators=200, random_state=1, n_jobs=-1),
    "KNN": KNeighborsClassifier(n_neighbors=5, metric="cosine", algorithm="brute", n_jobs=-1),
}
Xt, Xs, yt, ys = train_test_split(X, y, test_size=0.25, random_state=1, stratify=y)
res = {}
print(f"{'Model':<22}{'CV acc':<10}{'Test acc'}")
for k, m in models.items():
    p = Pipeline([("p", pre()), ("m", m)])
    cv = cross_val_score(p, Xt, yt, cv=5, n_jobs=-1).mean()
    p.fit(Xt, yt)
    pr = p.predict(Xs)
    res[k] = (cv, accuracy_score(ys, pr), p, pr)
    print(f"{k:<22}{cv:<10.3f}{res[k][1]:.3f}")
best = max(res, key=lambda k: res[k][0])
print("Selected:", best)
print(confusion_matrix(ys, res[best][3]))
ConfusionMatrixDisplay.from_predictions(ys, res[best][3])
plt.title(f"{best} ({res[best][1]:.2f})")
plt.savefig("ml_confusion.png", dpi=150)
final = Pipeline([("p", pre()), ("m", models[best])])
final.fit(X, y)
joblib.dump(final, "urgency_model.joblib")
tests = [("blood bags for emergency surgery", "Surgery", 14), ("bring lunch trays", "Ward", 12),
         ("antibiotics", "ICU", 10), ("antibiotics", "Pharmacy", 11),
         ("oxygen cylindr", "Ward", 3), ("pillows", "ICU", 23)]
for d, s, h in tests:
    q = pd.DataFrame({"d": [d], "s": [s], "n": [int(h < 6 or h >= 22)]})
    print(f"{d:<34}{s:<10}{h:<4} -> {final.predict(q)[0]}")
print("Saved urgency_model.joblib, ml_confusion.png")