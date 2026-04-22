# 🦾 دليل الإعداد الكامل — نظام الذراع الروبوتية الذكي
## Robotic Arm AI System — Complete Manual Setup Guide
### Ubuntu 24.04 LTS | ROS 2 Jazzy Jalopy | MoveIt 2

---

> ✅ **كل أمر في هذا الملف يُنسخ ويُنفَّذ مباشرة في التيرمينال.**  
> ✅ **الترتيب مهم جداً — لا تتخطَّ أي خطوة.**

---

## 📋 جدول المحتويات

1. [متطلبات النظام](#-1-متطلبات-النظام)
2. [المكتبات التي سيتم تنزيلها](#-2-المكتبات-التي-سيتم-تنزيلها)
3. [تثبيت ROS 2 Jazzy](#-3-تثبيت-ros-2-jazzy)
4. [تثبيت MoveIt 2](#-4-تثبيت-moveit-2)
5. [تثبيت ros2_control](#-5-تثبيت-ros2_control)
6. [تثبيت مكتبات الرؤية الحاسوبية](#-6-تثبيت-مكتبات-الرؤية-الحاسوبية)
7. [تثبيت مكتبات Python عبر pip + venv](#-7-تثبيت-مكتبات-python-عبر-pip--venv)
8. [إعداد Arduino والأذونات](#-8-إعداد-arduino-والأذونات)
9. [بناء المشروع](#-9-بناء-المشروع)
10. [أوامر التشغيل](#-10-أوامر-التشغيل)
11. [هيكل ملفات المشروع](#-11-هيكل-ملفات-المشروع)

---

## 💻 1. متطلبات النظام

| المكوّن | الحد الأدنى |
|---------|------------|
| نظام التشغيل | **Ubuntu 24.04 LTS** (لن يعمل على إصدارات أخرى) |
| المعالج | Intel Core i3 / AMD Ryzen 3 (64-bit) |
| الذاكرة RAM | 4 جيجابايت كحد أدنى (8 جيجابايت موصى) |
| كرت الشاشة | أي كرت يدعم OpenGL 4.0+ (لـ RViz2) |
| منافذ USB | 2 منافذ مستقلة (واحد للكاميرا وواحد للأردوينو) |
| الكاميرا | أي ويب كام USB |
| الأردوينو | Arduino Mega 2560 أو Arduino Uno |

---

## 📦 2. المكتبات التي سيتم تنزيلها

### أ) عبر `apt` (مستودعات Ubuntu + ROS 2)

| المكتبة | سبب الحاجة إليها |
|---------|-----------------|
| `ros-jazzy-desktop-full` | نواة ROS 2 كاملة مع RViz2 |
| `python3-colcon-common-extensions` | أداة بناء الـ packages في ROS 2 |
| `python3-rosdep` | إدارة تبعيات ROS تلقائياً |
| `ros-dev-tools` | أدوات تطوير ROS الأساسية |
| `ros-jazzy-moveit` | إطار التخطيط الحركي MoveIt 2 |
| `ros-jazzy-moveit-configs-utils` | أدوات إعداد MoveIt (يستخدمه launch files) |
| `ros-jazzy-moveit-ros-move-group` | خادم التخطيط الرئيسي (move_group node) |
| `ros-jazzy-moveit-ros-planning-interface` | واجهة برمجية لـ MoveIt من Python/C++ |
| `ros-jazzy-moveit-ros-visualization` | عرض مسارات الحركة في RViz |
| `ros-jazzy-moveit-simple-controller-manager` | وسيط بين MoveIt والمتحكمات |
| `ros-jazzy-moveit-kinematics` | حل عكس الكينماتيك (KDL solver) |
| `ros-jazzy-ros2-control` | نظام التحكم في الوقت الفعلي |
| `ros-jazzy-ros2-controllers` | مجموعة المتحكمات الجاهزة |
| `ros-jazzy-joint-trajectory-controller` | متحكم المسار التتابعي للمفاصل |
| `ros-jazzy-joint-state-broadcaster` | بث حالة المفاصل على ROS topics |
| `ros-jazzy-joint-state-publisher` | نشر حالة المفاصل يدوياً |
| `ros-jazzy-joint-state-publisher-gui` | واجهة رسومية لتحريك المفاصل يدوياً |
| `ros-jazzy-robot-state-publisher` | تحديث مواضع عناصر الروبوت (TF tree) |
| `ros-jazzy-tf2-ros` | نظام التحويلات الهندسية |
| `ros-jazzy-xacro` | معالج ملفات XACRO (امتداد URDF) |
| `ros-jazzy-rviz2` | برنامج العرض الثلاثي الأبعاد |
| `ros-jazzy-kdl-parser` | محلل kinematic chains من URDF |
| `ros-jazzy-cv-bridge` | جسر بين OpenCV و ROS topics |
| `ros-jazzy-image-transport` | نقل الصور عبر ROS بكفاءة |
| `ros-jazzy-image-geometry` | عمليات هندسية على صور الكاميرا |
| `python3-opencv` | مكتبة الرؤية الحاسوبية OpenCV |
| `python3-numpy` | حسابات المصفوفات والرياضيات |
| `python3-serial` | تواصل مع Arduino عبر Serial/USB |
| `python3-tk` | واجهة التحكم اليدوي (Tkinter GUI) |

### ب) عبر `pip` داخل `venv` (بيئة Python معزولة)

| المكتبة | الإصدار | سبب الحاجة |
|---------|---------|-----------|
| `opencv-python` | ≥ 4.6.0 | Fallback لو نسخة apt قديمة |
| `numpy` | ≥ 1.24.0 | Fallback لو نسخة apt قديمة |
| `pyserial` | ≥ 3.5 | تواصل mع Arduino عبر /dev/ttyACM0 |

---

## 🛜 3. تثبيت ROS 2 Jazzy

### الخطوة 3.1 — تحديث النظام
```bash
sudo apt update
sudo apt upgrade -y
```

### الخطوة 3.2 — إعداد اللغة (Locale)
```bash
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**التحقق:**
```bash
locale
# يجب أن يظهر: LANG=en_US.UTF-8
```

### الخطوة 3.3 — إضافة مستودع ROS 2
```bash
sudo apt install software-properties-common curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

### الخطوة 3.4 — تثبيت ROS 2 Jazzy Desktop كامل
```bash
sudo apt install ros-jazzy-desktop-full -y
```
⏳ هذا الأمر قد يأخذ **10-20 دقيقة** حسب سرعة الإنترنت.

### الخطوة 3.5 — تثبيت أدوات البناء
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep ros-dev-tools -y
```

### الخطوة 3.6 — تهيئة rosdep
```bash
sudo rosdep init
rosdep update
```

### الخطوة 3.7 — تفعيل ROS دائماً في التيرمينال
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**التحقق من نجاح التثبيت:**
```bash
ros2 --version
# يجب أن يظهر: ros2 jazzy ...

ros2 doctor
# يجب أن يظهر: All 5 checks passed
```

---

## 🤖 4. تثبيت MoveIt 2

```bash
sudo apt install \
    ros-jazzy-moveit \
    ros-jazzy-moveit-ros \
    ros-jazzy-moveit-core \
    ros-jazzy-moveit-kinematics \
    ros-jazzy-moveit-planners \
    ros-jazzy-moveit-planners-ompl \
    ros-jazzy-moveit-ros-move-group \
    ros-jazzy-moveit-ros-planning \
    ros-jazzy-moveit-ros-planning-interface \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-configs-utils \
    ros-jazzy-moveit-simple-controller-manager \
    ros-jazzy-moveit-setup-assistant \
    -y
```

**التحقق:**
```bash
ros2 pkg list | grep moveit
# يجب أن يظهر عشرات الأسطر التي تبدأ بـ moveit
```

---

## ⚙️ 5. تثبيت ros2_control

```bash
sudo apt install \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-kdl-parser \
    -y
```

---

## 👁️ 6. تثبيت مكتبات الرؤية الحاسوبية

```bash
sudo apt install \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-image-geometry \
    ros-jazzy-image-tools \
    python3-opencv \
    python3-numpy \
    python3-serial \
    python3-tk \
    python3-pip \
    -y
```

**التحقق:**
```bash
python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"
# يجب أن يظهر: OpenCV version: 4.6.0 أو أحدث

python3 -c "import numpy; print('NumPy version:', numpy.__version__)"
# يجب أن يظهر: NumPy version: 1.24.0 أو أحدث

python3 -c "import serial; print('PySerial OK')"
# يجب أن يظهر: PySerial OK
```

---

## 🐍 7. تثبيت مكتبات Python عبر pip + venv

### ما هو الـ venv ولماذا نستخدمه؟
الـ **venv** (Virtual Environment) هو بيئة Python معزولة داخل المشروع.  
- يمنع التعارض بين مكتبات النظام ومكتبات المشروع.
- يتيح لك تثبيت مكتبات بدون صلاحيات `sudo`.
- كل مكتبة pip تُنصَّب **داخل المجلد** وليس على مستوى النظام.

> ⚠️ **ملاحظة مهمة:** في ROS 2، الـ venv يُستخدم فقط للتطوير والاختبار.  
> **لا تُفعِّل الـ venv عند تشغيل الـ launch files** — فقط عند الاختبار اليدوي.

### الخطوة 7.1 — إنشاء الـ venv داخل المشروع
```bash
# انتقل إلى مجلد المشروع
cd ~/roboticarmproject/

# إنشاء البيئة الافتراضية باسم .venv
python3 -m venv .venv

# اعلم أنه سيتم إنشاء هذه المجلدات:
# project_robotic_arm_new/
# └── .venv/
#     ├── bin/       ← يحتوي على python, pip, activate
#     ├── lib/       ← المكتبات المنصَّبة
#     └── include/
```

### الخطوة 7.2 — تفعيل الـ venv
```bash
# في كل مرة تريد استخدام pip يدوياً، نفِّذ هذا أولاً:
source ~/roboticarmproject/.venv/bin/activate

# ستلاحظ أن التيرمينال يتغير ليظهر:
# (.venv) ahmed@ubuntu:~$
```

### الخطوة 7.3 — تثبيت المكتبات داخل الـ venv
```bash
# بعد تفعيل venv فقط:
pip install opencv-python>=4.6.0
pip install numpy>=1.24.0
pip install pyserial>=3.5
```

أو دفعة واحدة من ملف requirements.txt:
```bash
pip install -r ~/roboticarmproject/ros2_workspace/src/robotic_arm_vision/requirements.txt
```

### الخطوة 7.4 — إلغاء تفعيل الـ venv
```bash
deactivate
```
بعد `deactivate` يعود التيرمينال لوضعه الطبيعي.

### ملف requirements.txt للمشروع
**مسار:** `ros2_workspace/src/robotic_arm_vision/requirements.txt`
```
opencv-python>=4.6.0
numpy>=1.24.0
pyserial>=3.5
```

**مسار:** `ros2_workspace/src/robotic_arm_hardware/requirements.txt`
```
pyserial>=3.5
```

---

## 🔌 8. إعداد Arduino والأذونات

### الخطوة 8.1 — إضافة المستخدم لمجموعة dialout
```bash
sudo usermod -aG dialout $USER
```
> ⚠️ **مهم:** بعد هذا الأمر يجب **تسجيل الخروج ثم الدخول مجدداً** حتى يسري المفعول.

**التحقق بعد إعادة الدخول:**
```bash
groups $USER
# يجب أن يظهر dialout في القائمة:
# ahmed : ahmed adm dialout cdrom ...
```

### الخطوة 8.2 — رفع البرنامج على Arduino

1. ثبِّت Arduino IDE:
```bash
sudo snap install arduino
```

2. افتح الملف:
```
project_robotic_arm_new/arduino_firmware/robotic_arm_firmware/robotic_arm_firmware.ino
```

3. في Arduino IDE:
   - **Tools > Board** → اختر `Arduino Mega 2560`
   - **Tools > Port** → اختر `/dev/ttyACM0`
   - اضغط **Upload** (زر السهم)

### الخطوة 8.3 — التحقق من الاتصال
```bash
# تأكد أن الأردوينو مكتشف:
ls /dev/ttyACM*
# يجب أن يظهر: /dev/ttyACM0

# اختبار الاستجابة:
python3 -c "
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=3)
time.sleep(2)
msg = s.readline().decode(errors='ignore').strip()
print('Arduino:', msg)
s.close()
"
# يجب أن يظهر: Arduino: SYSTEM_BOOT_COMPLETE
```

---

## 🔨 9. بناء المشروع

### الخطوة 9.1 — الانتقال لمجلد الـ workspace
```bash
cd ~/roboticarmproject/ros2_workspace
```

### الخطوة 9.2 — تفعيل ROS 2
```bash
source /opt/ros/jazzy/setup.bash
```

### الخطوة 9.3 — تنزيل تبعيات ROS تلقائياً
```bash
rosdep install --from-paths src --ignore-src -r -y
```
هذا الأمر يقرأ ملفات `package.xml` ويثبت أي تبعية ناقصة.

### الخطوة 9.4 — بناء كل الـ packages
```bash
colcon build --symlink-install
```

**المخرج المتوقع:**
```
Starting >>> robotic_arm_description
Starting >>> robotic_arm_brain
Starting >>> robotic_arm_hardware
Starting >>> robotic_arm_teleop
Starting >>> robotic_arm_vision
Starting >>> robotic_arm_visualization
Finished <<< robotic_arm_description [1.77s]
Starting >>> robotic_arm_moveit_config
Finished <<< robotic_arm_moveit_config [1.72s]
Finished <<< robotic_arm_teleop [4.09s]
Finished <<< robotic_arm_brain [4.12s]
Finished <<< robotic_arm_hardware [4.16s]
Finished <<< robotic_arm_vision [4.24s]
Starting >>> robotic_arm_bringup
Finished <<< robotic_arm_bringup [1.33s]

Summary: 8 packages finished [5.94s]
```

### الخطوة 9.5 — تفعيل الـ workspace
```bash
source install/setup.bash
```

### الخطوة 9.6 — (اختياري) تفعيل دائم في .bashrc
```bash
echo "source ~/roboticarmproject/ros2_workspace/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**التحقق من نجاح البناء:**
```bash
ros2 pkg list | grep robotic_arm
```
يجب أن تظهر هذه الـ 8 packages:
```
robotic_arm_brain
robotic_arm_bringup
robotic_arm_description
robotic_arm_hardware
robotic_arm_moveit_config
robotic_arm_teleop
robotic_arm_vision
robotic_arm_visualization
```

---

## 🚀 10. أوامر التشغيل

> قبل أي أمر تشغيل، دائماً افتح تيرمينالاً جديداً ونفِّذ:
> ```bash
> cd ~/roboticarmproject/ros2_workspace
> source install/setup.bash
> ```

---

### وضع 1: 🤖 الذكاء الاصطناعي الكامل (الوضع الرئيسي)
```bash
ros2 launch robotic_arm_bringup ai_full_system.launch.py
```
- يفتح: نافذة **RViz2** (التجسيد ثلاثي الأبعاد) + نافذة **الكاميرا الحية**
- الذراع يبحث عن القطع الحمراء تلقائياً ويلتقطها
- بدون الأردوينو: يعمل في وضع المحاكاة فقط (RViz بدون حركة حقيقية)

**لتغيير لون الهدف من Red إلى Blue مثلاً:**
```bash
ros2 launch robotic_arm_bringup ai_full_system.launch.py
# في تيرمينال آخر:
ros2 param set /vision_perception_node target_color BLUE
```
**الألوان المدعومة:** `RED`, `BLUE`, `YELLOW`, `GREEN`

---

### وضع 2: 🕹️ التحكم اليدوي
```bash
ros2 launch robotic_arm_bringup manual_control.launch.py
```
- يفتح: RViz2 + واجهة رسومية للتحكم بكل مفصل يدوياً

---

### وضع 3: 👁️ عرض النموذج فقط (بدون أردوينو)
```bash
ros2 launch robotic_arm_bringup display.launch.py
```
- يفتح: RViz2 مع شريط تحريك المفاصل
- لا يحتاج كاميرا ولا أردوينو

---

### بناء package محدد فقط (أسرع للتطوير)
```bash
# بناء وحدة الدماغ فقط
colcon build --symlink-install --packages-select robotic_arm_brain
source install/setup.bash

# بناء وحدة الرؤية فقط
colcon build --symlink-install --packages-select robotic_arm_vision
source install/setup.bash

# بناء وحدة الهاردوير فقط
colcon build --symlink-install --packages-select robotic_arm_hardware
source install/setup.bash
```

---

### إعادة البناء من الصفر (عند حدوث أخطاء غريبة)
```bash
cd ~/roboticarmproject/ros2_workspace
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🗂️ 11. هيكل ملفات المشروع

```
project_robotic_arm_new/
│
├── SYSTEM_SETUP.md                  ← هذا الملف (دليل الإعداد الكامل)
│
├── .venv/                           ← بيئة Python المعزولة (pip)
│   ├── bin/activate                 ← أمر تفعيلها: source .venv/bin/activate
│   └── lib/                         ← المكتبات المثبتة بـ pip
│
├── arduino_firmware/
│   └── robotic_arm_firmware/
│       └── robotic_arm_firmware.ino ← ⬆️ يُرفع على الأردوينو
│
└── ros2_workspace/
    ├── src/                         ← كود المصدر
    │
    │   ├── robotic_arm_description/ ← نموذج URDF ثلاثي الأبعاد
    │   │   └── urdf/arm_robot.urdf  ← ملف هيكل الروبوت
    │
    │   ├── robotic_arm_moveit_config/ ← إعدادات MoveIt 2
    │   │   └── config/
    │   │       ├── arm_robot.srdf    ← تعريف مجموعات المفاصل
    │   │       ├── kinematics.yaml   ← إعدادات KDL solver
    │   │       ├── joint_limits.yaml ← حدود حركة المفاصل
    │   │       └── ros2_controllers.yaml ← إعدادات المتحكمات
    │
    │   ├── robotic_arm_bringup/     ← ملفات الإطلاق (Launch Files)
    │   │   └── launch/
    │   │       ├── ai_full_system.launch.py  ← ⭐ الأهم — تشغيل كل شيء
    │   │       ├── manual_control.launch.py  ← تحكم يدوي
    │   │       └── display.launch.py         ← عرض فقط
    │
    │   ├── robotic_arm_brain/       ← 🧠 العقل المدبر (FSM + Geometric IK)
    │   │   └── robotic_arm_brain/
    │   │       └── task_orchestrator_node.py ← منطق Pick & Place الكامل
    │
    │   ├── robotic_arm_vision/      ← 👁️ نظام الرؤية الحاسوبية
    │   │   ├── requirements.txt     ← مكتبات pip هذا الـ package
    │   │   └── robotic_arm_vision/
    │   │       └── vision_perception_node.py ← كشف الألوان + حساب الإحداثيات
    │
    │   ├── robotic_arm_hardware/    ← 🔌 التواصل مع الأردوينو عبر Serial
    │   │   ├── requirements.txt     ← مكتبات pip هذا الـ package
    │   │   └── robotic_arm_hardware/
    │   │       └── node_serial_bridge.py ← قراءة joint_states وإرسالها للأردوينو
    │
    │   ├── robotic_arm_teleop/      ← 🕹️ واجهة التحكم اليدوي (Tkinter GUI)
    │   │   └── robotic_arm_teleop/
    │   │       └── gui_teleop_node.py ← نافذة التحكم اليدوي
    │
    │   └── robotic_arm_visualization/ ← RViz مساعد (اختياري)
    │
    ├── build/   ← ملفات البناء المؤقتة (لا تعدِّل فيها)
    ├── install/ ← الـ packages المُنشأة (لا تعدِّل فيها)
    └── log/     ← سجلات البناء
```

---

## ❓ مشاكل شائعة وحلولها

### ❌ المشكلة: `Package 'robotic_arm_bringup' not found`
```bash
# الحل: مصدر الـ workspace قبل التشغيل
source ~/roboticarmproject/ros2_workspace/install/setup.bash
```

### ❌ المشكلة: `Cannot open ANY camera (tried 0-4)`
```bash
# تحقق من وجود الكاميرا
ls /dev/video*

# اختبر الكاميرا مباشرة
python3 -c "import cv2; c=cv2.VideoCapture(0); print('Camera open:', c.isOpened()); c.release()"
```

### ❌ المشكلة: `Reconnecting to /dev/ttyACM0...` (الأردوينو لا يتصل)
```bash
# تحقق من الجهاز
ls -la /dev/ttyACM*

# تحقق من عضوية dialout
groups $USER

# إذا لم تكن في dialout، أعِد تطبيق الأمر ثم سجِّل خروجاً ودخولاً:
sudo usermod -aG dialout $USER
# ثم سجِّل خروجاً من الجلسة ودخل مجدداً
```

### ✅ طبيعي — ليس خطأ: `Segmentation fault (exit code -11)` عند الإغلاق
هذه مشكلة معروفة في ROS 2 Jazzy عند إغلاق move_group. **لا تؤثر على التشغيل.**

### ✅ طبيعي — ليس خطأ: `Missing gear_left_joint, gripper_right_joint...`
هذه مفاصل تروس الجريبر الداخلية — مجرد بصرية ولا تتحكم فيها.

---

*آخر تحديث: April 2026*  
*النظام: Ubuntu 24.04 LTS | ROS 2 Jazzy Jalopy | MoveIt 2 | Python 3.12*

---
---

# 🟠 قسم Ubuntu 22.04 LTS — ROS 2 Humble Hawksbill
## نفس المشروع — بيئة مختلفة

> ⚠️ **الفرق الوحيد** بين Jazzy و Humble هو:
> - `ros-jazzy-*` تصبح `ros-humble-*`
> - Ubuntu **22.04** (Jammy) بدلاً من 24.04 (Noble)
> - Python المدمج هو **3.10** بدلاً من 3.12
> - بعض package names مختلفة قليلاً (موضحة أدناه)

---

## 🌐 H-1. تثبيت ROS 2 Humble

### H-1.1 — تحديث النظام
```bash
sudo apt update
sudo apt upgrade -y
```

### H-1.2 — إعداد اللغة
```bash
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### H-1.3 — إضافة مستودع ROS 2 Humble
```bash
sudo apt install software-properties-common curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```
> `$UBUNTU_CODENAME` سيكون `jammy` تلقائياً على Ubuntu 22.04

### H-1.4 — تثبيت ROS 2 Humble Desktop كامل
```bash
sudo apt install ros-humble-desktop-full -y
```

### H-1.5 — أدوات البناء
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep ros-dev-tools -y
```

### H-1.6 — تهيئة rosdep
```bash
sudo rosdep init
rosdep update
```

### H-1.7 — تفعيل ROS دائماً
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**التحقق:**
```bash
ros2 --version
# يجب أن يظهر: ros2 humble ...
```

---

## 🤖 H-2. تثبيت MoveIt 2 (Humble)

```bash
sudo apt install \
    ros-humble-moveit \
    ros-humble-moveit-ros \
    ros-humble-moveit-core \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-planners \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-setup-assistant \
    -y
```

---

## ⚙️ H-3. تثبيت ros2_control (Humble)

```bash
sudo apt install \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-trajectory-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-kdl-parser \
    -y
```

---

## 👁️ H-4. مكتبات الرؤية الحاسوبية (Humble)

```bash
sudo apt install \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-image-geometry \
    ros-humble-image-tools \
    python3-opencv \
    python3-numpy \
    python3-serial \
    python3-tk \
    python3-pip \
    -y
```

---

## 🐍 H-5. بيئة Python (venv) — Humble

الخطوات **مطابقة تماماً** لنسخة Jazzy:

```bash
# إنشاء الـ venv
cd ~/roboticarmproject/
python3 -m venv .venv

# تفعيل
source .venv/bin/activate

# تثبيت المكتبات
pip install opencv-python>=4.6.0
pip install numpy>=1.24.0
pip install pyserial>=3.5

# إلغاء التفعيل
deactivate
```

---

## 🔌 H-6. إعداد Arduino (Humble)

مطابق تماماً لقسم [8. إعداد Arduino](#-8-إعداد-arduino-والأذونات) أعلاه:

```bash
sudo usermod -aG dialout $USER
# سجِّل خروجاً ودخولاً بعد هذا الأمر
```

---

## 🔨 H-7. بناء المشروع (Humble)

```bash
cd ~/roboticarmproject/ros2_workspace

# تفعيل Humble بدلاً من Jazzy
source /opt/ros/humble/setup.bash

# تنزيل التبعيات
rosdep install --from-paths src --ignore-src -r -y

# البناء
colcon build --symlink-install

# تفعيل الـ workspace
source install/setup.bash
```

### تفعيل دائم في .bashrc (Humble)
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/roboticarmproject/ros2_workspace/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**التحقق:**
```bash
ros2 pkg list | grep robotic_arm
# يجب أن تظهر 8 packages
```

---

## 🚀 H-8. أوامر التشغيل (Humble)

أوامر التشغيل **مطابقة تماماً** — لا يوجد أي فرق:

```bash
# وضع الذكاء الاصطناعي الكامل
ros2 launch robotic_arm_bringup ai_full_system.launch.py

# وضع التحكم اليدوي
ros2 launch robotic_arm_bringup manual_control.launch.py

# وضع العرض فقط
ros2 launch robotic_arm_bringup display.launch.py
```

---

## 📊 مقارنة سريعة: Jazzy vs Humble

| العنصر | Ubuntu 24.04 + Jazzy | Ubuntu 22.04 + Humble |
|--------|---------------------|----------------------|
| اسم النظام | Noble Numbat | Jammy Jellyfish |
| اسم ROS | Jazzy **Jalopy** | Humble **Hawksbill** |
| Python | **3.12** | **3.10** |
| اسم الـ packages | `ros-jazzy-*` | `ros-humble-*` |
| مسار ROS | `/opt/ros/jazzy/` | `/opt/ros/humble/` |
| دعم حتى | **May 2029** | **May 2027** |
| أوامر التشغيل | ✅ مطابقة | ✅ مطابقة |
| كود المشروع | ✅ مطابق | ✅ مطابق |

> 💡 **التوصية:** استخدم **Ubuntu 24.04 + Jazzy** لأنه أحدث ومدعوم حتى 2029.  
> استخدم **Ubuntu 22.04 + Humble** فقط إذا كان الجهاز لا يدعم Ubuntu 24.04.

---

*آخر تحديث: April 2026*  
*Ubuntu 24.04 | ROS 2 Jazzy | MoveIt 2 | Python 3.12*  
*Ubuntu 22.04 | ROS 2 Humble | MoveIt 2 | Python 3.10*
