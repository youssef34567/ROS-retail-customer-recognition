import sqlite3

# Connect to SQLite database
conn = sqlite3.connect('people.db')
cursor = conn.cursor()

# Create table for storing face information
cursor.execute('''
CREATE TABLE IF NOT EXISTS people (
    name TEXT PRIMARY KEY,
    info TEXT
)
''')

# Add initial data
cursor.executemany('''
INSERT OR IGNORE INTO people (name, info) VALUES (?, ?)
''', [
    ('Bejad', 'Bejad, Uniqlo is offering up to 50% off, along with new arrivals tailored to your previous purchases that you will love!'),
    ('Youssef', 'Youssef, H&M has exclusive discounts of up to 30%, along with new arrivals you will adore'),
    ('John Cena', 'John Cena, Cold Stone Creamery is celebrating Black Friday with sweet discounts! Enjoy up to 40% off your favourite ice cream creations a treat you wont want to miss!'),
    ('Unknown', 'Would you like to register for our program to be informed when your favorite shops have discounts or new arrivals')
])

# Save changes and close the connection
conn.commit()
conn.close()

print("Database created and populated successfully!")
