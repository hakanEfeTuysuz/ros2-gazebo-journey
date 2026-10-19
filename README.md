# ros2-gazebo-journey

A step-by-step learning journey and project workspace for ROS 2 and Gazebo on Ubuntu.

Bu repo, **ROS 2 Humble + Gazebo Classic** üzerinde diferansiyel sürüşlü bir robotun SLAM ile çıkarılan haritada **Nav2** ile otonom gezinmesini içerir.

## Ortam

| Bileşen | Sürüm |
|---|---|
| ROS 2 | Humble |
| Simülatör | Gazebo Classic |
| Navigasyon | `nav2_bringup` (AMCL + DWB + NavFn) |
| SLAM | slam_toolbox (harita bir kez çıkarıldı, `maps/` içinde) |

Gerekli paketler:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-gazebo-ros-pkgs ros-humble-xacro
```

## Klasör yapısı

```
src/
├── my_robot_description/
│   ├── urdf/               my_robot.urdf.xacro, lidar.xacro
│   ├── worlds/             test_world.world (5.8 x 5.8 m oda, 2 silindir, 1 bölme)
│   ├── maps/               my_world_map.yaml / .pgm
│   ├── config/             nav2_params.yaml
│   ├── behavior_trees/     navigate_to_pose_backup_first.xml
│   ├── launch/             gazebo.launch.py
│   └── robot_view.rviz
└── my_robot_core/          obstacle_avoidance.cpp (Nav2'den bağımsız basit engel kaçınma)
```

## Kurulum ve çalıştırma

```bash
cd ~/ros2-gazebo-journey
colcon build --symlink-install
source install/setup.bash
```

Her terminalde `source install/setup.bash` yapılmalıdır.

**Terminal 1: Gazebo**

```bash
ros2 launch my_robot_description gazebo.launch.py
```

**Terminal 2: Nav2**

```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true \
  map:=$HOME/ros2-gazebo-journey/src/my_robot_description/maps/my_world_map.yaml \
  params_file:=$HOME/ros2-gazebo-journey/src/my_robot_description/config/nav2_params.yaml
```

**Terminal 3: RViz**

```bash
ros2 launch nav2_bringup rviz_launch.py
```

> `nav2_params.yaml` içindeki `default_nav_to_pose_bt_xml` yolu **mutlak** yazılmıştır
> (`/home/hakanefe/...`). Farklı bir kullanıcı/makinede bu satırı kendi yoluna göre değiştir.

`my_robot_core/obstacle_avoidance` node'u Nav2 ile **aynı anda çalıştırılmamalıdır** (bkz. Notlar).

## Robot geometrisi

- Gövde: 0.5 x 0.3 x 0.1 m, 2 kg
- Tekerlekler: yarıçap 0.06 m, iz genişliği 0.34 m
- `base_footprint` **tekerlek aksının orta noktasının yer izdüşümüdür**; `base_link` aksın 0.15 m önündedir
- Kaster (küre) önde, sürtünmesi neredeyse sıfır
- Lidar: `laser_frame`, 360 ışın, 0.12 – 8.0 m, 10 Hz
- Nav2 footprint (`base_footprint` çerçevesinde): `[[0.40, 0.19], [0.40, -0.19], [-0.10, -0.19], [-0.10, 0.19]]`

TF zinciri: `map → odom → base_footprint → base_link → laser_frame`

## Yapılan iyileştirmeler

| # | Sorun | Neden | Çözüm | Doğrulama |
|---|---|---|---|---|
| 1 | Robot yerinde dönerken konumu kayıyor, hedefte ileri-geri salınıyor | `base_footprint` gövde merkezindeydi, gerçek dönme merkezi (aks) 0.15 m gerideydi | `base_joint` `x=0.15` yapıldı, footprint aksa taşındı | `tf2_echo base_footprint left_wheel` → x=0; yerinde dönüşte sapma ~5 mm |
| 2 | Kaster zemine yapışıyor, robot sarsılıyor | `<mu1 value="..."/>` Gazebo'da geçersiz yazım | `<mu1>0.001</mu1>` biçimine çevrildi | Gazebo'da gözle kontrol |
| 3 | Gazebo ve Nav2 ivme limitleri uyumsuz | `max_wheel_acceleration` çok düşük, `max_wheel_torque` çok yüksek | torque 5, ivme 10 rad/s², update_rate 50 | Nav2 limitlerinden gevşek |
| 4 | Robotun arkasında hayalet engeller | Lidar şasi üstünden 2 cm yükseklikte, ışınlar kendi şasisine çarpıyordu (arkada 0.35 m'de) | Lidar `z=0.07 → 0.10` | Arka ışınlar kayboldu |
| 5 | Haritada tek hücrelik hayalet engel | SLAM gürültüsü, (x=0.87, y=-2.14) | Hücre temizlendi | Piksel analizi |
| 6 | Dar geçişlerde robot geçemiyor | `inflation_radius 0.55` fazla büyüktü, 1.1 m'lik boşluklar tamamen kapanıyordu | `inflation_radius 0.40`, `cost_scaling_factor 8.0` | Aynı hedef tekrar verilince geçti |
| 7 | Robot köşelere/duvara vuruyor | DWB `BaseObstacle` kriteri sadece merkez noktasına bakıyor, gövde köşelerine bakmıyor | `ObstacleFootprint` kriterine geçildi | Önceki vurmalar azaldı |
| 8 | Sıkışınca geri çıkamıyor | DWB geri gitmiyor; varsayılan kurtarma sırasında BackUp en sondaydı ve çok yavaştı | Özel davranış ağacı: BackUp ilk sırada (0.30 m, 0.10 m/s). DWB'de `min_vel_x` 0.0 bırakıldı, yani normal sürüşte geri gitmez | Kurtarma sadece takılınca çalışır |
| 9 | Frame tutarsızlığı | AMCL `base_footprint`, diğerleri `base_link` | Hepsi `base_footprint` | TF zinciri |
| 10 | `config` / `maps` kurulmuyordu | `CMakeLists.txt` `install` komutunda yoktu | `config`, `maps`, `behavior_trees`, `robot_view.rviz` eklendi | `ls install/.../share/my_robot_description/` |

Diğer: hız limitleri tutarlı hale getirildi (`max_vel_x 0.20`, `max_vel_theta 1.0`, ivme 0.5 / 1.5), AMCL lidar menzili gerçek değerlere (0.12 – 8.0 m) çekildi, hedef toleransı 0.15 m yapıldı.

## Geliştirilecekler

- **Köşe hedefleri:** Duvar ya da köşe kenarına verilen hedeflerde robot bazen yaklaşıp takılabiliyor. Hedef toleransları ve planlayıcı `tolerance` değeri bu duruma göre ayarlanacak.
- **Dönme yarıçapı:** Aks gövdenin biraz gerisinde olduğu için yerinde dönerken ön köşe geniş bir yay çiziyor. Gerekirse tekerlekler gövde ortasına alınıp arkaya ikinci bir kaster eklenecek.
- **Gerçek robota geçiş:** `odometry_source = 1` (Gazebo gerçek konumu) sadece simülasyon içindir. Gerçek robotta encoder odometrisine geçilecek.

## Notlar

- RViz'de **Nav2 Goal** verirken oku sürükleyerek robotun bakacağı yönü seç. Köşe hedeflerinde oku odanın içine doğru çevirmek en rahatı.
- `my_robot_core/obstacle_avoidance` node'u Nav2 ile aynı anda çalıştırılmamalıdır, ikisi de `/cmd_vel`'e yazar.
- `nav2_params.yaml` içindeki davranış ağacı yolu mutlaktır, farklı bir makinede kendi yoluna göre değiştirilmelidir.