import datetime
import joblib
import pandas as pd
M = joblib.load("urgency_model.joblib")
def predict(d, s, h):
    q = pd.DataFrame({"d": [d.lower()], "s": [s], "n": [int(h < 6 or h >= 22)]})
    p = int(M.predict(q)[0])
    c = None
    try:
        c = float(M.predict_proba(q).max())
    except Exception:
        c = None
    return p, c
def ask_tasks(base, dests):
    lk = {n.lower(): n for n in dests}
    hr = datetime.datetime.now().hour
    t = input(f"Hour 0-23 (Enter = now, {hr}): ").strip()
    if t.isdigit() and 0 <= int(t) <= 23:
        hr = int(t)
    tasks = [dict(x) for x in base]
    print("\nExisting tasks:")
    for x in tasks:
        print(f"  priority {x['priority']}  {x['name']} -> {x['dest']}")
    print("\nDestinations:", ", ".join(dests))
    while True:
        s = input("New task destination (Enter to finish): ").strip().lower()
        if s == "":
            break
        if s not in lk:
            print("  unknown destination, try again")
            continue
        d = input("Item description: ").strip()
        if d == "":
            print("  description needed, task skipped")
            continue
        p, c = predict(d, lk[s], hr)
        tasks.append({"name": f"T{len(tasks)+1} {d} to {lk[s]}", "dest": lk[s], "priority": p})
        if c is None:
            print(f"  ML urgency: {p}")
        else:
            print(f"  ML urgency: {p} (confidence {c:.0%})")
    return tasks