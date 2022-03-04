
# Dijkstra_path_planning
Implemented Dijkstra algorithm to plan a path between two points

## Result
* Blue circle represents the start location 
* Red circle represents the end location
<img src="https://github.com/mjoshi07/Dijkstra_path_planning/blob/main/Data/viz.gif" height=300>

## Map Generation
* Using half-plane equations, we generate obstacles
* In particular, for the current problem, generated 1 concave polygon, 1 hexagon and 1 circle
* Obstacle is represented with 0 `black` pixel value and free space as 255 `white`
<img src="https://github.com/mjoshi07/Dijkstra_path_planning/blob/main/Data/sample_final_map.png" height=300>

## Run Instructions
* If you already have the binary world map, then rename it as final_map.png, else you can generate the map first as well
* Run the following command to start the dijkstra solver
```
python main.py -h
```
* You will see the following options
```
optional arguments:
 -h, --help            show this help message and exit
 --init INIT            start location of the robot. Example - 10,12 [NOTE - without spaces], Default: Random
 --goal GOAL            end location of the robot. Example - 150,120 [NOTE - without spaces], Default: Random
  --clearance CLEARANCE clearance dist from obstacles, DEFAULT: 5
```
* --init takes input the start location of the robot: Input should be comma separated without any spaces, for example: 10,12
* --goal takes input the end location of the robot: Input should be comma separated without any spaces, for example: 10,12
* --clearance takes input the min distance to maintain for the robot from all of the obstacles boundaries
* By default, if user does not specify any location for start or end, random locations would be selected
* By default, clearance is set to 5 and cannot be set more than 50
* If "final_map.png" file does not exists, then program will ask user whether to generate a new file
* The program also asks the user whether to display the progress of the map being generated using half planes techniques
* If you want to specify only the start location, modify and run the command below, end location would be generated randomly
```
python main.py --init 50,50
```
* If you want to specify only the end location, modify and run the command below, start location would be generated randomly
```
python main.py --goal 50,50
```
* If you want to specify both the start and end location, modify and run the command below
```
python main.py --init 100,100 --goal 50,50
```
* If you want to specify the clearance for the robot, modify and run the command below, start and end location would be generated randomly
```
python main.py --clearance 10
```
* If you want to specify everything
```
python main.py --init 100,100 --goal 50,50 --clearance 10
```
* If the init and goal location are in the obstacle the program would notify the user and exit

