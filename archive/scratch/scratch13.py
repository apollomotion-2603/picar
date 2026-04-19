import sqlite3

def explore_bag(db_path):
    conn = sqlite3.connect(db_path)
    c = conn.cursor()
    c.execute("SELECT t.name, t.type FROM topics t")
    rows = c.fetchall()
    print(f"Topics in {db_path}:")
    for r in rows:
        print(f"  {r[0]} ({r[1]})")

explore_bag('bag_map1_test_3/bag_map1_test_3_0.db3')
