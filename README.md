# ROS2_PRM*_Project

## Opis projektu:
Ten kod jest częścią projektu "Nav2 PRM* Planner" i jest odpowiedzialny za implementację planera nawigacyjnego opartego na algorytmie PRM* (Probabilistic Roadmap Planner) oraz algorytmie A* (A-star). Projekt jest rozwijany w ramach platformy ROS (Robot Operating System) i ma na celu umożliwienie robotom autonomicznym wyznaczania trajektorii w złożonym środowisku.

Kod zawiera klasę StraightLine, która implementuje funkcje potrzebne do konfiguracji planera, generowania losowych punktów, tworzenia roadmapy, przeszukiwania grafu za pomocą algorytmu A* oraz konstruowania optymalnej ścieżki.

## Główne etapy działania planera PRM* to:

1. **Konfiguracja:** W ramach tej fazy planera ustala się parametry, takie jak rozmiar mapy, liczba wierzchołków roadmapy, promień sąsiedztwa, maksymalna liczba sąsiadów itp. Wszystkie te parametry mogą być dostosowane do konkretnych wymagań aplikacji.

2. **Generowanie wierzchołków:** Planer losowo generuje wierzchołki w przestrzeni roboczej, unikając kolizji z przeszkodami. Punkty te reprezentują potencjalne lokalizacje, które robot może odwiedzić.

3. **Tworzenie roadmapy:** Na podstawie wygenerowanych wierzchołków tworzona jest roadmapa, czyli graf skierowany, w którym wierzchołki reprezentują położenie robota, a krawędzie reprezentują połączenia między tymi położeniami. Wykorzystywany jest algorytmy szukający sąsiadów w określonym promieniu.

4. **Wykrywanie połączeń:** Dla każdego wierzchołka sprawdzane jest, czy istnieje połączenie z innym wierzchołkiem, które jest wolne od kolizji. Wykorzystywane są algorytmy sprawdzające kolizje, aby stwierdzić, czy istnieje widoczność między dwoma punktami.

5. **Przeszukiwanie grafu**: Po utworzeniu roadmapy, algorytm A* jest wykorzystywany do wyszukiwania optymalnej ścieżki między punktem startowym a docelowym. Algorytm A* korzysta z funkcji heurystycznej (np. odległość Euklidesowa) do oszacowania kosztu pozostałego do celu i wybiera najbardziej obiecujące wierzchołki do eksploracji.

6. **Konstrukcja ścieżki:** Na podstawie informacji o połączeniach między wierzchołkami w grafie, konstruowana jest optymalna ścieżka, która prowadzi od punktu startowego do docelowego.

Algorytm PRM* jest używany w systemach robotycznych opartych na ROS Navigation i znajduje zastosowanie w różnych scenariuszach, takich jak nawigacja wewnątrz budynków, eksploracja nieznanych obszarów czy planowanie trasy dla robotów mobilnych.

## Implementacja
1. Początkowo stworzono implementacje algorytmu PRM* w jezyku python który pozwala na łatwiejsze posługiwanie się składowymi:
![image](https://github.com/pawel-gawron/ROS2_PRM_star_Project/assets/65308689/06a11fc5-9c12-4727-a56f-f87dc972e816)

2. Następnie kod został zaimplementowany na docelowym środowisku jakim jest nav2 napisany za pomocą języka C++:
![image](https://github.com/pawel-gawron/ROS2_PRM_star_Project/assets/65308689/e9073312-ff1b-42a0-8eb9-04ba4bbca1e4)

## Przydatne odnośniki
1. https://navigation.ros.org/getting_started/index.html#running-the-example
2. https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html

## Sposób włączenia programu
### Python
cd ../ROS2_PRM_star_Project
colcon build
source /opt/ros/<ros2-distro>/setup.bash
source install/setup.bash
ros2 launch mapr_6_student rrt_launch.py vertices:=True

### C++
cd ../ROS2_PRM_star_Project
colcon build
source /opt/ros/<ros2-distro>/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py


