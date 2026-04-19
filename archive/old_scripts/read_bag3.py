import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def get_messages(db_path, topic_name):
    conn = sqlite3.connect(db_path)
    c = conn.cursor()
    c.execute("SELECT id, type FROM topics WHERE name = ?", (topic_name,))
    row = c.fetchone()
    if not row:
        print(f"Topic {topic_name} not found.")
        return
    topic_id, msg_type = row
    
    msg_class = get_message(msg_type)
    
    c.execute("SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,))
    rows = c.fetchall()
    
    print(f"--- Data from {db_path} ---")
    kappas = []
    for ts, data in rows:
        msg = deserialize_message(data, msg_class)
        kappas.append(msg.kappa)
        
    print(f"Total msgs: {len(kappas)}")
    if len(kappas) > 0:
        print(f"Kappa: min={min(kappas):.4f}, max={max(kappas):.4f}, sum={sum(kappas):.4f}, avg={sum(kappas)/len(kappas):.4f}")
        print(f"First 10 Kappas: {[round(k, 4) for k in kappas[:10]]}")

get_messages('bag_map1_test_3/bag_map1_test_3_0.db3', '/perception/lane_state')
