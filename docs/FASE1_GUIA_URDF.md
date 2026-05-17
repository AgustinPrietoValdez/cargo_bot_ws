# Fase 1: Guia para crear el URDF/Xacro del cargo_bot

## Orden de aprendizaje recomendado

### Paso 1: Tutoriales oficiales ROS 2 (leelos en orden)
1. [Building a Visual Robot Model](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html) -- links, joints, geometrias visuales
2. [Building a Movable Robot Model](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Movable-Robot-Model-with-URDF.html) -- joints continuous/revolute, limites
3. [Using Xacro](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html) -- variables, macros, math, includes
4. [URDF con Robot State Publisher](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html) -- publicar TF desde URDF

### Paso 2: Guia Nav2 (convenciones de frames)
- [Nav2: Setting Up The URDF](https://docs.nav2.org/setup_guides/urdf/setup_urdf.html) -- explica base_link, base_footprint, odom frame

### Paso 3: Ejemplo de referencia (estudialo a fondo)
- [joshnewans/articubot_one](https://github.com/joshnewans/articubot_one) -- robot diferencial completo, multi-file xacro, ros2_control, lidar, camera
- Video/blog del autor: [Articulated Robotics: URDF Design](https://articulatedrobotics.xyz/tutorials/mobile-robot/concept-design/concept-urdf/)

### Paso 4: Fusion 360 → meshes para URDF
- [ACDC4Robot plugin](https://github.com/ACDC4Robot/Fusion360) -- exporta URDF directo desde Fusion 360 (recomendado)
- [runtimerobotics/fusion360-urdf-ros2](https://github.com/runtimerobotics/fusion360-urdf-ros2) -- alternativa que genera un paquete ROS 2 completo
- Video: [Articulated Robotics: Concept & URDF Design](https://articulatedrobotics.xyz/tutorials/mobile-robot/concept-design/concept-urdf/)

### Paso 5: Antes de importar a Isaac Sim
- [Isaac Sim 5.1: Import URDF Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/import_urdf.html)

---

## Flujo Fusion 360 → URDF → Isaac Sim

### Opcion A: Plugin automatico (recomendado para empezar rapido)

1. Instalar [ACDC4Robot](https://github.com/ACDC4Robot/Fusion360) en Fusion 360 (tambien en [Autodesk App Store](https://apps.autodesk.com/FUSION/en/Detail/Index?id=5028052292896011577))
2. El plugin exporta URDF + meshes STL directamente
3. Importar el URDF en Isaac Sim

### Opcion B: Manual (mas control, mejor para aprender)

#### Preparar el CAD en Fusion 360

1. **Un componente por link URDF** -- cada pieza que se mueve independientemente (chasis, rueda izq, rueda der, caster, etc.) debe ser un Component separado en Fusion
2. **NO usar componentes anidados** -- todos los link-components deben ser hijos directos del root assembly
3. **Nombrar bien** -- `base_link`, `left_wheel`, `right_wheel`, `caster_wheel`, `lidar_mount`. Solo letras, numeros, underscore
4. **Definir joints en Fusion** -- usar la herramienta Joint para definir Revolute (ruedas), Rigid (caster, lidar). Los ejes de rotacion deben coincidir con los ejes reales
5. **Orientar el robot** -- ROS usa X-adelante, Y-izquierda, Z-arriba. Fusion usa Y-arriba por defecto. Reorientar antes de exportar
6. **Origen en el centro entre ruedas** -- el origen del ensamble debe estar en el punto medio entre las dos ruedas motrices, a nivel del suelo

#### Exportar meshes

1. **Click derecho en cada componente > "Save as STL"** (NO usar File > Export, que siempre exporta en cm)
2. **Refinamiento High** para meshes visuales, **Low** para colision
3. Fusion trabaja en mm, URDF espera metros. Hay dos opciones:
   - Poner document units en metros antes de exportar
   - Dejar en mm y agregar `scale="0.001 0.001 0.001"` en cada tag `<mesh>` del URDF
4. Guardar en:
   ```
   cargo_bot_description/meshes/
   ├── visual/       # STL detallados (High refinement)
   │   ├── base_link.stl
   │   ├── left_wheel.stl
   │   ├── right_wheel.stl
   │   └── ...
   └── collision/    # STL simplificados (Low refinement, o primitivas)
       ├── base_link.stl
       └── ...
   ```

#### Referenciar en Xacro

```xml
<link name="base_link">
  <visual>
    <geometry>
      <mesh filename="package://cargo_bot_description/meshes/visual/base_link.stl"
            scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
  <collision>
    <!-- Opcion 1: mesh simplificado -->
    <geometry>
      <mesh filename="package://cargo_bot_description/meshes/collision/base_link.stl"
            scale="0.001 0.001 0.001"/>
    </geometry>
    <!-- Opcion 2 (recomendado para rendimiento): primitiva -->
    <!--
    <geometry>
      <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
    </geometry>
    -->
  </collision>
  <xacro:inertial_box mass="${chassis_mass}"
                       x="${chassis_length}" y="${chassis_width}" z="${chassis_height}"/>
</link>
```

#### Obtener masas e inercias de Fusion 360

1. Seleccionar el componente
2. Menu: Inspect > Physical Properties
3. Anotar: Mass (convertir a kg), Center of Mass, Moments of Inertia (convertir a kg*m^2)
4. Usar esos valores en los tags `<inertial>` del URDF, o usar las macros de abajo que los calculan de la geometria

### Tip: colision visual vs colision fisica

| Mesh | Uso | Formato | Detalle |
|------|-----|---------|---------|
| Visual | Lo que se ve en RViz/Isaac | STL High | Detallado, el peso no importa |
| Colision | Lo que la fisica usa para chocar | Primitiva o STL Low | Lo mas simple posible, afecta performance |

Para el cargo_bot, recomiendo **primitivas para colision** (cajas y cilindros) y **STL de Fusion para visual**. Asi se ve tu robot real pero la fisica corre rapido.

---

## Estructura de archivos que vas a crear

```
cargo_bot_ws/src/cargo_bot_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── cargo_bot.urdf.xacro        # Archivo principal (incluye los demas)
│   ├── inertial_macros.xacro       # Macros para calcular inercias
│   ├── chassis.xacro               # base_link (chasis)
│   ├── wheels.xacro                # ruedas motrices + caster
│   └── sensors.xacro               # lidar_link, imu_link
├── meshes/                          # STLs de tu CAD (opcional al principio)
├── rviz/
│   └── display.rviz
└── launch/
    └── display.launch.py            # Lanza robot_state_publisher + RViz2
```

---

## TF Tree que debe resultar

```
base_footprint (en el suelo, proyeccion de base_link)
  └── base_link (centro del chasis)
        ├── left_wheel_link   (joint: continuous, eje Y)
        ├── right_wheel_link  (joint: continuous, eje Y)
        ├── caster_wheel_link (joint: fixed, esfera)
        ├── lidar_link        (joint: fixed, arriba del chasis)
        └── imu_link          (joint: fixed, centro del chasis)
```

Cuando Nav2 corre, agrega: `map -> odom -> base_footprint`

---

## Formulas de inercia (las vas a necesitar)

### Caja (chasis) -- dimensiones x, y, z; masa m
```
ixx = m * (y^2 + z^2) / 12
iyy = m * (x^2 + z^2) / 12
izz = m * (x^2 + y^2) / 12
```

### Cilindro (ruedas) -- radio r, altura h; masa m
```
ixx = m * (3*r^2 + h^2) / 12
iyy = m * (3*r^2 + h^2) / 12
izz = m * r^2 / 2
```
NOTA: En URDF los cilindros van en Z. Para ruedas (giran en Y), rotarlas 90 grados en el `<origin>` del joint.

### Esfera (caster) -- radio r; masa m
```
ixx = iyy = izz = 2 * m * r^2 / 5
```

### Macro Xacro (copiar y adaptar)
```xml
<xacro:macro name="inertial_box" params="mass x y z">
  <inertial>
    <mass value="${mass}"/>
    <inertia ixx="${(1/12)*mass*(y*y+z*z)}" ixy="0.0" ixz="0.0"
             iyy="${(1/12)*mass*(x*x+z*z)}" iyz="0.0"
             izz="${(1/12)*mass*(x*x+y*y)}"/>
  </inertial>
</xacro:macro>

<xacro:macro name="inertial_cylinder" params="mass length radius">
  <inertial>
    <mass value="${mass}"/>
    <inertia ixx="${(1/12)*mass*(3*radius*radius+length*length)}" ixy="0.0" ixz="0.0"
             iyy="${(1/12)*mass*(3*radius*radius+length*length)}" iyz="0.0"
             izz="${(1/2)*mass*radius*radius}"/>
  </inertial>
</xacro:macro>

<xacro:macro name="inertial_sphere" params="mass radius">
  <inertial>
    <mass value="${mass}"/>
    <inertia ixx="${(2/5)*mass*radius*radius}" ixy="0.0" ixz="0.0"
             iyy="${(2/5)*mass*radius*radius}" iyz="0.0"
             izz="${(2/5)*mass*radius*radius}"/>
  </inertial>
</xacro:macro>
```

---

## Gotchas de Fusion 360

1. **File > Export da centimetros** -- siempre usar "Save as STL" (click derecho en component), no File > Export
2. **Componentes, no Bodies** -- cada link URDF debe ser un Component de Fusion, no solo un Body
3. **Sin anidamiento** -- los plugins no soportan componentes anidados. Aplanar la jerarquia
4. **Y-up vs Z-up** -- Fusion usa Y-arriba, ROS usa Z-arriba. Reorientar antes de exportar o rotar en el URDF
5. **Fusion calcula inercias** -- Inspect > Physical Properties te da masa, centro de masa e inercias. Usalas

## Gotchas importantes para Isaac Sim

1. **Xacro no se importa directo** -- Primero convertir: `ros2 run xacro xacro cargo_bot.urdf.xacro > cargo_bot.urdf`
2. **Nombres sin caracteres especiales** -- Solo letras, numeros, underscore
3. **Todo link necesita masa + inercia** -- Si falta, la fisica explota
4. **Stage Units Per Meter = 1.0** -- Verificar ANTES de importar (Edit > Preferences > Stage > Meters per unit)
5. **Wheel joints = Velocity drive** -- Despues de importar, cambiar el drive type de las ruedas a Velocity (stiffness=0, damping=alto)
6. **Marcar "Import Inertia Tensor"** -- Para usar tus valores calculados, no los auto-calculados
7. **Self-collision = OFF** -- A menos que verifiques que no hay meshes intersectando
8. **Caster wheel** -- Joint fixed + esfera con friction=0 (mu_static=0, mu_dynamic=0 en Isaac Sim physics material)
9. **Correr Asset Validator** despues de importar: Window > Asset Validator

---

## Convenciones para cargo_bot

| Parametro | Variable Xacro | Valor (completar con tu CAD) |
|-----------|---------------|------------------------------|
| Largo chasis | `chassis_length` | ? m |
| Ancho chasis | `chassis_width` | ? m |
| Alto chasis | `chassis_height` | ? m |
| Masa chasis (con carga) | `chassis_mass` | ~7.5 kg (robot + 5 kg) |
| Radio rueda | `wheel_radius` | ? m |
| Ancho rueda | `wheel_width` | ? m |
| Masa por rueda | `wheel_mass` | ? kg |
| Separacion entre ruedas (track) | `wheel_separation` | ? m (eje a eje) |
| Offset caster (desde base_link) | `caster_offset_x` | ? m |
| Radio caster | `caster_radius` | ? m |
| Altura lidar (desde base_link) | `lidar_height` | ? m |

---

## Repos de referencia adicionales

| Repo | Para que sirve |
|------|---------------|
| [joshnewans/articubot_one](https://github.com/joshnewans/articubot_one) | Arquitectura xacro completa (PRINCIPAL) |
| [ros2_control_demos (DiffBot)](https://github.com/ros-controls/ros2_control_demos) | Ejemplo oficial ros2_control |
| [TheNoobInventor/lidarbot](https://github.com/TheNoobInventor/lidarbot) | Robot real con RPLidar + RPi4 |
| [DimitrisKatos/dd_robot](https://github.com/DimitrisKatos/dd_robot) | Educativo paso a paso |

---

## Checklist antes de pasar a Fase 2

- [ ] CAD completo en Fusion 360 con un Component por link URDF
- [ ] Meshes STL exportados (visual/ y collision/ o usar primitivas para colision)
- [ ] Si exportaste en mm: `scale="0.001 0.001 0.001"` en cada `<mesh>` del URDF
- [ ] URDF compila sin errores: `ros2 run xacro xacro cargo_bot.urdf.xacro`
- [ ] Se visualiza en RViz2: `ros2 launch cargo_bot_description display.launch.py`
- [ ] El robot se ve igual a tu CAD (meshes cargados correctamente)
- [ ] Ruedas giran con joint_state_publisher_gui
- [ ] TF tree es correcto (verificar con `ros2 run tf2_tools view_frames`)
- [ ] Todas las colisiones son geometrias simples (cajas, cilindros, esferas) o STL Low
- [ ] Todos los links tienen masa + inercia (de Fusion Physical Properties o calculada)
- [ ] Nombres de joints/links sin caracteres raros (solo letras, numeros, _)
- [ ] Exportado a .urdf puro (sin xacro) listo para importar en Isaac Sim:
      `ros2 run xacro xacro cargo_bot.urdf.xacro > cargo_bot.urdf`
