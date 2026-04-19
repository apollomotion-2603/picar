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
    msg = deserialize_message(rows[0][1], msg_class)
    print(f"Message:")
    print(f" kappa = {msg.kappa}")
    print(f" coeff_a = {msg.coeff_a}")
    print(f" coeff_b = {msg.coeff_b}")
    print(f" coeff_c = {msg.coeff_c}")
    print(f" coeff_d = {msg.coeff_d}")
    print(f" e_psi = {msg.e_psi}")
    print(f" e_y = {msg.e_y}")

get_messages('bag_map1_test_3/bag_map1_test_3_0.db3', '/perception/lane_state')
