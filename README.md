Pada projek ini dilakukan penerapan Image-Based Visual Servo (IBVS) pada Robot Dog lite-3 dan Tangan OpenManipulator.

## 1. Arsitektur Komunikasi Sistem
Sistem berjalan terpusat pada **NVIDIA Jetson Xavier NX** yang bertindak sebagai "otak", menghubungkan dua subsistem hardware utama:

* **Robot Dog (Kaki):** Menerima perintah kecepatan dan mode gerak.
* **OpenManipulator (Tangan):** Menerima target sudut sendi (joint angles) untuk pergerakan.

### Alur Data Utama:
1.  **Input Visual:** Citra dari *Kamera Depan* dan *Kamera Gripper* masuk ke Jetson.
2.  **Processing (Python Node):** Jetson memproses gambar, menghitung error posisi target, dan menentukan aksi selanjutnya.
3.  **Command Output:**
    * Navigasi Jarak Jauh: Mengirim data ke `move_base`.
    * Koreksi Posisi (Tracking): Mengirim `cmd_vel` (linear/angular) ke robot dog.
    * Eksekusi Grasping: Mengirim sinyal trigger boolean ke node C++ (`navman`).

---

## 2. Mekanisme Alur Kerja (Berdasarkan Flowchart)

Mekanisme robot dibagi menjadi 4 fase utama sesuai dengan perubahan *State Machine*:

### FASE 1: Navigasi Global (State: NAVIGATING)
* **Mekanisme:** User memilih tujuan di Web Dashboard.
* **Proses:** Node Python mengirim koordinat (x, y) ke *Move Base*. Robot menggunakan sensor Lidar/Odometri untuk berjalan otomatis menghindari rintangan menuju titik tersebut.
* **Hasil:** Robot sampai di lokasi dan masuk ke mode `SEARCHING` (diam menunggu input).

### FASE 2: Visual Servoing Jarak Jauh (State: TRACKING)
* **Mekanisme:** Operator menggambar kotak (Bounding Box) pada objek di video stream *Kamera Depan*.
* **Proses:**
    * Sistem mengunci objek menggunakan algoritma CSRT Tracker.
    * Menghitung titik tengah objek ($C_x, C_y$).
    * Menggunakan **PID Controller** untuk memutar robot (agar $C_x$ di tengah layar) dan memajukan robot.
* **Transisi:** Ketika objek terdeteksi sudah sangat dekat (melewati batas bawah layar), robot berhenti otomatis.

### FASE 3: Manuver Rotasi & Transisi (State: MANEUVER)
* **Masalah:** Lengan robot terpasang di punggung/samping, sehingga tidak bisa mengambil objek yang tepat berada di depan wajah robot.
* **Mekanisme:**
    1.  Robot melakukan rotasi badan 90 derajat (Hardcoded/IMU based).
    2.  Sistem mematikan *Kamera Depan* dan mengaktifkan *Kamera Gripper* (kamera di ujung tangan).
    3.  Lengan bergerak ke posisi siaga (`POSE_ROTATE_LEFT`).

### FASE 4: Penyelarasan & Eksekusi (State: ALIGNING -> GRASPING)
* **Alignment (Penyelarasan):**
    * Menggunakan *Kamera Gripper*, robot melihat objek dari atas/samping.
    * Robot melakukan gerakan geser samping hingga gripper hingga tegak lurus dengan objek.
    * Setelah posisi pas, robot mengirim perintah **Duduk (SIT)** untuk mengunci posisi agar stabil.
* **Handover ke C++:**
    * Operator melakukan konfirmasi final di Web.
    * Python mengirim sinyal `True` ke topik `/navman_comm`.
    * **Node C++** mengambil alih kendali penuh untuk menghitung *Inverse Kinematics* (menggerakkan sendi lengan turun) dan menutup gripper.

---

## 3. State Machine

| Kode State | Nama State | Fungsi Utama | Kamera Aktif |
| :--- | :--- | :--- | :--- |
| `0` | **IDLE** | Robot diam, menunggu perintah misi. | Body |
| `1` | **NAVIGATING** | Robot berjalan ke lokasi peta (Autonomous). | Body |
| `3` | **TRACKING** | Mengejar objek visual dari jauh. | Body |
| `4` | **MANEUVER** | Robot berputar 90° untuk memposisikan lengan. | - |
| `5` | **ALIGNING** | Robot geser samping untuk meluruskan gripper. | Gripper |
| `6` | **READY** | Robot duduk, menunggu konfirmasi operator. | Gripper |
| `7` | **GRASPING** | Logika C++ mengambil alih untuk menjepit objek. | - |


Berikut adalah gambar Arsitektur sistem dan Sequence Diagram dari Robot Dog

![Arsitektur Sistem](/images/ArsitekturSistem.png)
![Flowchart](/images/Flowchart.png)
![Sequence Diagram](/images/sequencediagram.jpeg)