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
│   │   ├── launch/             bringup.launch.py (hepsi), gazebo.launch.py, navigation.launch.py
│   │   └── robot_view.rviz
│   └── my_robot_core/          obstacle_avoidance.cpp (Nav2'den bağımsız basit engel kaçınma)
└── tools/
    ├── nav_goal_test.py        Otomatik Nav2 hedef testi (bkz. Test)
    └── run_sim_test.sh         Headless simülasyon + test, tek komut
```

## Kurulum ve çalıştırma

```bash
cd ~/ros2-gazebo-journey
colcon build --symlink-install
source install/setup.bash
```

Her terminalde `source install/setup.bash` yapılmalıdır.

**Tek komut (Gazebo + Nav2 + RViz):**

```bash
ros2 launch my_robot_description bringup.launch.py
```

Argümanlar: `gui:=false` (Gazebo penceresiz), `rviz:=false`.

Parçalar ayrı ayrı da açılabilir:

```bash
ros2 launch my_robot_description gazebo.launch.py        # gui, world, x, y, yaw
ros2 launch my_robot_description navigation.launch.py    # map, params_file, use_sim_time
ros2 launch nav2_bringup rviz_launch.py
```

Tüm yollar (harita, parametreler, davranış ağacı) paket dizininden bulunur, mutlak yol yoktur.
Nav2'yi yeniden başlatırken Gazebo'yu da yeniden başlatmak en temizidir: AMCL başlangıç pozunu (0, 0) olarak bekler.

`my_robot_core/obstacle_avoidance` node'u Nav2 ile **aynı anda çalıştırılmamalıdır** (bkz. Notlar).

## Robot geometrisi

- Gövde: 0.5 x 0.3 x 0.1 m, 2 kg
- Tekerlekler: yarıçap 0.06 m, iz genişliği 0.34 m
- `base_footprint` **tekerlek aksının orta noktasının yer izdüşümüdür**; `base_link` aksın 0.15 m önündedir
- Kaster (küre) önde, sürtünmesi neredeyse sıfır
- Lidar: `laser_frame`, 360 ışın (1°), 0.12 – 8.0 m, 10 Hz, σ = 1 cm gauss gürültü
- Odometri: tekerlek encoder'ı (`odometry_source 0`); gerçek konum sadece test için `/ground_truth`
- Nav2 footprint (`base_footprint` çerçevesinde): `[[0.40, 0.19], [0.40, -0.19], [-0.10, -0.19], [-0.10, 0.19]]`
- Yerinde dönerken ön köşenin çizdiği yarıçap: yaklaşık 0.44 m

TF zinciri: `map → odom → base_footprint → base_link → laser_frame`

## Yapılan iyileştirmeler

| # | Sorun | Neden | Çözüm | Doğrulama |
|---|---|---|---|---|
| 1 | Robot yerinde dönerken konumu kayıyor, hedefte ileri-geri salınıyor | `base_footprint` gövde merkezindeydi, gerçek dönme merkezi (aks) 0.15 m gerideydi | `base_joint` `x=0.15` yapıldı, footprint aksa taşındı | `tf2_echo base_footprint left_wheel` → x=0; yerinde dönüşte sapma ~5 mm |
| 2 | Kaster zemine yapışıyor, robot sarsılıyor | `<mu1 value="..."/>` Gazebo'da geçersiz yazım | `<mu1>0.001</mu1>` biçimine çevrildi | Gazebo'da gözle kontrol |
| 3 | Gazebo ve Nav2 ivme limitleri uyumsuz | Tekerlek ivmesi 5 rad/s² × 0.06 m = 0.3 m/s², DWB 0.5 m/s² varsayıyordu | torque 10, ivme 10 rad/s² (0.6 m/s²), update_rate 50 | Nav2 limitlerinden gevşek |
| 4 | Robotun arkasında hayalet engeller | Lidar şasi üstünden 2 cm yükseklikte, ışınlar kendi şasisine çarpıyordu (arkada 0.35 m'de) | Lidar `z=0.07 → 0.10` | Arka ışınlar kayboldu |
| 5 | Haritada tek hücrelik hayalet engel | SLAM gürültüsü, (x=0.87, y=-2.14) | Hücre temizlendi | Piksel analizi |
| 6 | Dar geçişlerde robot geçemiyor | Büyük `inflation_radius` 1.1 m'lik boğazları kapatıyordu | Global costmap: `inflation_radius 0.45`, `cost_scaling_factor 6.0`. Local costmap: `0.40` / `8.0` | Boğaz genişliği haritadan hesaplandı (1.10 m), temiz bant 0.20 m |
| 7 | Robot köşelere/duvara vuruyor | DWB `BaseObstacle` kriteri sadece merkez noktasına bakıyor, gövde köşelerine bakmıyor | `ObstacleFootprint` kriterine geçildi | Önceki vurmalar azaldı |
| 8 | Sıkışınca geri çıkamıyor | DWB geri gitmiyor; varsayılan kurtarma sırasında BackUp en sondaydı ve çok yavaştı | Özel davranış ağacı: BackUp ilk sırada (0.30 m, 0.10 m/s). DWB'de `min_vel_x` 0.0 bırakıldı, yani normal sürüşte geri gitmez | Kurtarma sadece takılınca çalışır |
| 9 | Frame tutarsızlığı | AMCL `base_footprint`, diğerleri `base_link` | Hepsi `base_footprint` | TF zinciri |
| 10 | `config` / `maps` kurulmuyordu | `CMakeLists.txt` `install` komutunda yoktu | `config`, `maps`, `behavior_trees`, `robot_view.rviz` eklendi | `ls install/.../share/my_robot_description/` |
| 11 | Köşe/duvar kenarı hedeflerinde robot takılıyor | Hedef, duvarın yasak bandına düşüyor | Planlayıcı `tolerance 0.5`. (Önceki `xy 0.30` / `yaw 3.14` geçici çözümü #16'da geri alındı) | Köşe hedefleri `Goal succeeded` ile tamamlanıyor |
| 12 | Takılma geç fark ediliyordu | `movement_time_allowance` 20 s | 8 s (#17'de yeniden ayarlandı) | Kurtarma daha erken başlıyor |
| 13 | DWB bir yöne kilitlenebiliyor | `Oscillation` kriterinde zaman bazlı sıfırlama kapalıydı | `Oscillation.oscillation_reset_time: 1.0` | Kaynak kod okundu |
| 14 | Proje başka makinede çalışmıyor | Davranış ağacı yolu yaml'da mutlak (`/home/hakanefe/...`) | `navigation.launch.py` yolu paket dizininden `RewrittenYaml` ile yazıyor | `ros2 param get /bt_navigator default_nav_to_pose_bt_xml` → `install/.../share/...` |
| 15 | Lokalizasyon test edilmiyordu | `odometry_source 1`: Gazebo kusursuz konum veriyordu, AMCL `alpha 0.01` buna göre | Encoder odometrisi (`0`), AMCL `alpha1-4: 0.2` | AMCL hatası gerçek konuma göre ≤ 0.15 m (test) |
| 16 | Robot hedefe ~0.3 m kala duruyor, yönü yok sayıyor | `xy_goal_tolerance 0.30`, `yaw_goal_tolerance 3.14` | `0.15` / `0.25` | Test: hedef hatası ~0.14 m, yön ≤ 0.26 rad |
| 17 | Robot sebepsiz "takılıyor", kurtarma davranışları başlıyor | Progress checker 8 s'de 0.5 m bekliyordu; 0.2 m/s robot yerinde dönüp manevra yaparken bunu yapamıyor → sahte `Failed to make progress` | `required_movement_radius 0.25`, `movement_time_allowance 10` | Test (aşağıda) |
| 19 | Robot giderken sağa-sola salınıyor | DWB `PathAlign`/`GoalAlign` `forward_point_distance 0.1`: yön çok yakın bir noktaya göre düzeltiliyordu | `0.325` (Nav2 varsayılanı) | Test: salınım 199 → 63 yön değişimi, `user_divider` FAIL → 19 s'de OK |
| 20 | Duvar/engel içine verilen hedeflerde robot dakikalarca uğraşıyor | Planlayıcı `tolerance 0.5` ulaşılamaz hedefe en yakın noktaya yol çiziyordu | `tolerance 0.25` | Kullanıcı oturum logları |
| 18 | Gazebo + Nav2 tek launch'ta çökmesi | gazebo_ros `gzserver.launch.py` da `params_file` (boş) tanımlıyor, launch yapılandırmaları global | `bringup.launch.py`'de her include `GroupAction` ile kapsamlı | Headless test |

## Geliştirilecekler

- ~~**Çubuğun arkasındaki hedefler**~~ (çözüldü): Uzun çubuğun arkasındaki renkli (inflation) bölgeye hedef verilince robot bazen yavaşlayıp duruyor. Beyaz bölgelerdeki hedefler sorunsuz çalışıyor. Asıl sebep progress checker'ın sahte "Failed to make progress" üretmesiydi (#17); düzeltmeden sonra testte bu hedefler kurtarmasız tamamlanıyor.
- **Duvar dibinde dönme:** Aks gövdenin gerisinde olduğu için yerinde dönerken ön köşe 0.44 m'lik yay çiziyor; duvara yakın hedeflerde robot hâlâ sağa-sola deneyerek dönüyor (testte `user_north_wall`). Tekerlekleri ortaya almak denendi: dönüş yarıçapı 0.31 m'ye indi ama arka çıkıntı büyüdüğü için robot engel dibinde sıkıştı; çalışması için DWB'nin geri gidebilmesi ve çarpışma payı ayarı gerekiyor.
- **Hedef seçimi:** Robot 0.38 m genişliğinde; RViz'de duvara ~0.3 m'den, engellere ~0.25 m'den yakın hedef vermek fiziksel olarak mümkün değil.
- **Engele yakın geçiş:** Zor hedeflerde gövde engele 5 cm'ye kadar yaklaşıyor (çarpmıyor). `ObstacleFootprint.scale` ve local inflation ayarıyla artırılabilir.
- **Gazebo Classic desteği bitti (Ocak 2025):** Uzun vadede Gazebo Harmonic + `ros_gz`'ye geçilmeli.

## Test

`tools/run_sim_test.sh` Gazebo'yu penceresiz açar, Nav2'yi başlatır ve `tools/nav_goal_test.py` ile 9 hedefi (2'si engele ~0.3 m yakın, zor) sırayla gönderir:

```bash
tools/run_sim_test.sh --rounds 2
```

Her hedef için sonuç, süre, kurtarma sayısı, sağ-sol salınım sayısı (`osc`, dönüş komutunun yön değiştirmesi), gövdenin engele en yakın mesafesi (`clear`), hedef hatası ve AMCL'nin gerçek konuma göre hatası (`loc_max`) yazılır. Loglar `log/sim_test/` altındadır.

Ölçülen sonuçlar (2 tur × 9 hedef):

| Ayar | Başarı | Kurtarma | Hedef hatası | En yakın gövde mesafesi |
|---|---|---|---|---|
| Eski ayarlar | 7/7 (zor hedefler yoktu) | 1 | ~0.30 m, yön yok sayılıyor | ölçülmüyordu |
| Toleranslar + encoder odometrisi | 18/18 | 5 | ~0.14 m / ≤0.26 rad | 0.04 m |
| MPPI kontrolcü (denendi, kullanılmadı) | 17/18 | 54 | ~0.14 m | 0.02 m |
| DWB + progress checker düzeltmesi | 18/18 | 1 | ~0.14 m / ≤0.26 rad | 0.05 m |

Salınım testi (1 tur × 11 hedef, kullanıcının zorlandığı 2 hedef eklendi):

| Ayar | Başarı | Kurtarma | Salınım (yön değişimi) |
|---|---|---|---|
| Önceki | 10/11 | 21 | 199 |
| **Şu anki: `forward_point_distance 0.325`, planlayıcı `tolerance 0.25`** | **11/11** | **2** | **63** |
| Tekerlekler gövde ortasında (denendi, kullanılmadı) | 9/11 | 29 | 345 |

## Notlar

- RViz'de **Nav2 Goal** verirken oku sürükleyerek robotun bakacağı yönü seç; robot hedefte bu yöne döner.
- `my_robot_core/obstacle_avoidance` node'u Nav2 ile aynı anda çalıştırılmamalıdır, ikisi de `/cmd_vel`'e yazar.
