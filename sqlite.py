#!/usr/bin/env python3

import rospy
import sqlite3
import threading
import os
from std_msgs.msg import String

class SQLiteNode:
    def __init__(self):
        rospy.init_node('sqlite_node', anonymous=True)

        # Subscriber to recognized_faces topic
        rospy.Subscriber('recognized_faces', String, self.query_database)

        # Publisher to send info to TTS Node
        self.info_pub = rospy.Publisher('person_info', String, queue_size=10)

        # Get absolute path to the database file
        db_path = os.path.join(os.path.dirname(__file__), 'people.db')
        rospy.loginfo(f"Using database file: {db_path}")

        # Connect to the database with thread safety
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.cursor = self.conn.cursor()

        # Lock for thread-safe database access
        self.db_lock = threading.Lock()
        rospy.loginfo("SQLite Node initialized!")

    def query_database(self, msg):
        """Query the database for the name and publish the result."""
        name = msg.data
        rospy.loginfo(f"Querying database for {name}...")

        with self.db_lock:  # Ensure only one thread accesses the DB at a time
            try:
                self.cursor.execute("SELECT info FROM people WHERE name=?", (name,))
                result = self.cursor.fetchone()

                if result:
                    info = result[0]
                    rospy.loginfo(f"Found info: {info}")
                    self.info_pub.publish(info)
                else:
                    rospy.logwarn(f"No information found for {name}")
            except sqlite3.Error as e:
                rospy.logerr(f"Database error: {e}")

    def close(self):
        """Close the database connection."""
        self.conn.close()


if __name__ == '__main__':
    try:
        node = SQLiteNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.close()
