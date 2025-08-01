import sqlite3

# 데이터베이스 파일에 연결 (없으면 자동 생성됨)
conn = sqlite3.connect('RfRobot.db')

# 커서(cursor) 객체 생성
cur = conn.cursor()

# 테이블 생성 (이미 있다면 생략 가능)
cur.execute('''
    CREATE TABLE IF NOT EXISTS users (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        age INTEGER
    )
''')

# 데이터 삽입
cur.execute("INSERT INTO users (name, age) VALUES (?, ?)", ('Alice', 30))

# 변경사항 저장
conn.commit()

# 데이터 조회
cur.execute("SELECT * FROM users")
rows = cur.fetchall()
for row in rows:
    print(row)

# 연결 종료
conn.close()
