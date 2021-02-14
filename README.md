# SoccerSimulator

[RoboCup Small Size League](https://ssl.robocup.org/) Simulator.

### Client Visualizer

<p align="center">
  Sample python client files exist in client folder!
<img src = "https://github.com/ErfanFathi/soccer-simulator/blob/main/client/util/img.png"</img>
</p>

### Description
This repository uses [grSim](https://github.com/RoboCup-SSL/grSim) physics core . this project
developed for high-performance tasks like ML and DL .<br>
see the [LICENSE](LICENSE) file for more information .

### Configs
Check the config file in ```$projectPath/config/config.h```

### Dependencies
- [CMake](https://cmake.org/) version 3.5+
- [Qt5 Development Libraries](https://www.qt.io)
- [Open Dynamics Engine (ODE)](http://www.ode.org)
- [Google Protobuf](https://github.com/google/protobuf)

### Compile
```bash
mkdir build
cd build
cmake ..
make
```

### Run 
```cd $projectPath/bin```<br>
```./soccerSim```

### Finale

Report anything to:<br>
fathierfan97@gmail.com<br>

### Citing
If you use grSim in your research, please cite [the following paper](http://link.springer.com/chapter/10.1007/978-3-642-32060-6_38):

> Monajjemi, Valiallah (Mani), Ali Koochakzadeh, and Saeed Shiry Ghidary. "grSim – RoboCup Small Size Robot Soccer Simulator." In Robot Soccer World Cup, pp. 450-460. Springer Berlin Heidelberg, 2011.
