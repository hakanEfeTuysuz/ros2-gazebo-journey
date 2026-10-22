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
├── src/
│   ├── my_robot_description/
│   │   ├── urdf/               my_robot.urdf.xacro, lidar.xacro
│   │   ├── worlds/             test_world.world (5.8 x 5.8 m oda, 2 silindir, 1 bölme)
│   │   ├── maps/               my_world_map.yaml / .pgm
│   │   ├── config/             nav2_params.yaml
│   │   ├── behavior_trees/     navigate_to_pose_backup_first.xml
│   │   ├── launch/             gazebo.launch.py
│   │   └── robot_view.rviz
│   └── my_robot_core/          obstacle_avoidance.cpp (Nav2'den bağımsız basit engel kaçınma)
└── tools/
    └── eval_check.py           DWB hata ayıklama aracı (bkz. Hata ayıklama)
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

Nav2'yi yeniden başlatırken Gazebo'yu da yeniden başlatmak en temizidir. AMCL başlangıç pozunu (0, 0) olarak bekler, robot başka bir yerdeyken Nav2'yi açmak konumlandırmayı bozar.

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
- Yerinde dönerken ön köşenin çizdiği yarıçap: yaklaşık 0.44 m

TF zinciri: `map → odom → base_footprint → base_link → laser_frame`

## Yapılan iyileştirmeler

| # | Sorun | Neden | Çözüm | Doğrulama |
|---|---|---|---|---|
| 1 | Robot yerinde dönerken konumu kayıyor, hedefte ileri-geri salınıyor | `base_footprint` gövde merkezindeydi, gerçek dönme merkezi (aks) 0.15 m gerideydi | `base_joint` `x=0.15` yapıldı, footprint aksa taşındı | `tf2_echo base_footprint left_wheel` → x=0; yerinde dönüşte sapma ~5 mm |
| 2 | Kaster zemine yapışıyor, robot sarsılıyor | `<mu1 value="..."/>` Gazebo'da geçersiz yazım | `<mu1>0.001</mu1>` biçimine çevrildi | Gazebo'da gözle kontrol |
| 3 | Gazebo ve Nav2 ivme limitleri uyumsuz | `max_wheel_acceleration` çok düşük, `max_wheel_torque` çok yüksek | torque 5, ivme 10 rad/s², update_rate 50 | Nav2 limitlerinden gevşek |
| 4 | Robotun arkasında hayalet engeller | Lidar şasi üstünden 2 cm yükseklikte, ışınlar kendi şasisine çarpıyordu (arkada 0.35 m'de) | Lidar `z=0.07 → 0.10` | Arka ışınlar kayboldu |
| 5 | Haritada tek hücrelik hayalet engel | SLAM gürültüsü, (x=0.87, y=-2.14) | Hücre temizlendi | Piksel analizi |
| 6 | Dar geçişlerde robot geçemiyor | Büyük `inflation_radius` 1.1 m'lik boğazları kapatıyordu | Global costmap: `inflation_radius 0.45`, `cost_scaling_factor 6.0`. Local costmap: `0.40` / `8.0` | Boğaz genişliği haritadan hesaplandı (1.10 m), temiz bant 0.20 m |
| 7 | Robot köşelere/duvara vuruyor | DWB `BaseObstacle` kriteri sadece merkez noktasına bakıyor, gövde köşelerine bakmıyor | `ObstacleFootprint` kriterine geçildi | Önceki vurmalar azaldı |
| 8 | Sıkışınca geri çıkamıyor | DWB geri gitmiyor; varsayılan kurtarma sırasında BackUp en sondaydı ve çok yavaştı | Özel davranış ağacı: BackUp ilk sırada (0.30 m, 0.10 m/s). DWB'de `min_vel_x` 0.0 bırakıldı, yani normal sürüşte geri gitmez | Kurtarma sadece takılınca çalışır |
| 9 | Frame tutarsızlığı | AMCL `base_footprint`, diğerleri `base_link` | Hepsi `base_footprint` | TF zinciri |
| 10 | `config` / `maps` kurulmuyordu | `CMakeLists.txt` `install` komutunda yoktu | `config`, `maps`, `behavior_trees`, `robot_view.rviz` eklendi | `ls install/.../share/my_robot_description/` |
| 11 | Köşe/duvar kenarı hedeflerinde robot takılıyor | Hedef, duvarın yasak bandına düşüyor, robot hedefin 0.15 m yakınına giremiyor | `xy_goal_tolerance 0.30`, planlayıcı `tolerance 0.5`, `yaw_goal_tolerance 3.14` (hedefte dönme kapalı) | Köşe hedefleri `Goal succeeded` ile tamamlanıyor |
| 12 | Takılma geç fark ediliyordu | `movement_time_allowance` 20 s | 8 s | Kurtarma daha erken başlıyor |
| 13 | DWB bir yöne kilitlenebiliyor | `Oscillation` kriterinde zaman bazlı sıfırlama kapalıydı | `Oscillation.oscillation_reset_time: 1.0` | Kaynak kod okundu |

## Geliştirilecekler

- **Çubuğun arkasındaki hedefler:** Uzun çubuğun arkasındaki renkli (inflation) bölgeye hedef verilince robot bazen yavaşlayıp duruyor. Beyaz bölgelerdeki hedefler sorunsuz çalışıyor. Hata ayıklamada DWB'nin 819 yolunun tamamının geçerli olduğu, ama "olduğun yerde kal" seçeneğinin ileri gitmekten yaklaşık 1 puan iyi çıktığı görüldü. `PreferForward` kriteri denendi (`scale 4.0`, `theta_scale 2.0`), tek başına yetmedi. Sıradaki: robotun engele gerçek mesafesinin kontrolü ve kriter ağırlıklarının ayarı.
- **Dönme yarıçapı:** Aks gövdenin biraz gerisinde olduğu için yerinde dönerken ön köşe geniş bir yay çiziyor. Gerekirse tekerlekler gövde ortasına alınıp arkaya ikinci bir kaster eklenecek.
- **Gerçek robota geçiş:** `odometry_source = 1` (Gazebo gerçek konumu) sadece simülasyon içindir. Gerçek robotta encoder odometrisine geçilecek.

## Hata ayıklama

`tools/eval_check.py`, DWB'nin o an denediği tüm yolları hız gruplarına ayırır, kaçının geçerli olduğunu, geçersizlerin hangi kriterle elendiğini ve kazanan yolun puan dökümünü yazar. Robot takılıyken çalıştırılır (`debug_trajectory_details: True` açık olmalıdır):

```bash
cd ~/ros2-gazebo-journey && source /opt/ros/humble/setup.bash && source install/setup.bash
python3 tools/eval_check.py
```

Kriter ağırlıkları `nav2_params.yaml` içindeki değerlerle birebir aynı değildir. `PathAlign`, `GoalAlign`, `PathDist`, `GoalDist` için DWB ölçeği çözünürlükle çarpar (yaml 32 → etkin 0.80, 24 → 0.60).

## Notlar

- RViz'de **Nav2 Goal** verirken oku sürükleyerek robotun bakacağı yönü seç. Hedefte dönme şu an kapalıdır (`yaw_goal_tolerance 3.14`), robot sadece konuma gider.
- `my_robot_core/obstacle_avoidance` node'u Nav2 ile aynı anda çalıştırılmamalıdır, ikisi de `/cmd_vel`'e yazar.
- `nav2_params.yaml` içindeki davranış ağacı yolu mutlaktır, farklı bir makinede kendi yoluna göre değiştirilmelidir.