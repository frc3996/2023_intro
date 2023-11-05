# Introduction a RobotPy!

Apprendre a programmer un projet Python utilisant RobotPy et le framework MagicBot

Presentement ce projet supporte:

- Le mode simulateur avec support pour un controleur xbox ou clavier
- Quelques commandes autonome
- Le Robot de pratique!
- DifferentialDrive avec moteur CANSparkMax
- Pneumatic Control Module

## Utilisation

Apres avoir suivi le guide d'Introduction disponible ici https://github.com/martinrioux/guide-demarrage-robotpy

**Installer tous les package de RobotPy**

```
# Linux/MacOS
python3 -m pip robotpy[all]

# Windows
py -3 -m pip install -U robotpy[all]
```

**Demarrer le mode simulateur**

```
# Linux/MacOS
python3 -m pip robotpy[all]

# Windows
py -3 -m pip robotpy[all]
```

## Installer et mettre a jour RobotPy sur le Robot (periodiquement)

### Installer Python (seulement apres une re-installation)

**A partir d'un reseau qui avec acces a Internet**

```
# Linux/MacOS
python3 -m robotpy_installer download-python

# Windows
py -3 -m robotpy_installer download-python
```

**A partir du reseau du Robot**

```
# Linux/MacOS
python3 -m robotpy_installer install-python

# Windows
py -3 -m robotpy_installer install-python
```

### Mettre a jour les packages de RobotPy (periodiquement)

**A partir d'un reseau qui avec acces a Internet**

```
# Linux/MacOS
python3 -m robotpy_installer download robotpy[all]

# Windows
py -3 -m robotpy_installer download robotpy[all]
```

**A partir du reseau du Robot**

```
# Linux/MacOS
python3 -m robotpy_installer install robotpy[all]

# Windows
py -3 -m robotpy_installer install robotpy[all]
```

## Deployer sur le Robot

**A partir du reseau du Robot**

```
#Linux
python3 robot.py deploy

# Windows
py -3 robot.py deploy
```

## References

A special thank you to the ElectricLights team #6367 for making their RobotPy projects available at https://github.com/frc6367
