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

## Referencia URDF/Xacro: cada elemento explicado

### Estructura basica de un archivo Xacro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="mi_robot">
  <!-- Todo el contenido va aca adentro -->
</robot>
```
- `xmlns:xacro` habilita las funciones de Xacro (variables, macros, math, includes)
- `name` es el nombre del robot (aparece en RViz, logs, etc.)

---

### `<xacro:property>` -- Variables

Definen constantes reutilizables. Cambias el valor en un solo lugar y se propaga a todo el archivo.

```xml
<xacro:property name="wheel_radius" value="0.04"/>
<xacro:property name="wheel_mass"   value="0.2"/>
```

Se usan con `${}`:
```xml
<cylinder radius="${wheel_radius}" length="${wheel_width}"/>
```

Soportan matematica:
```xml
<origin xyz="${chassis_length/2} 0 0"/>
<origin xyz="0 ${wheel_separation/2} 0"/>
```

---

### `<xacro:include>` -- Incluir otros archivos

Divide el URDF en archivos mas chicos y manejables:

```xml
<xacro:include filename="inertial_macros.xacro"/>
<xacro:include filename="chassis.xacro"/>
<xacro:include filename="wheels.xacro"/>
<xacro:include filename="sensors.xacro"/>
```

Cada archivo incluido tiene su propio `<robot xmlns:xacro=...>` wrapper pero se fusionan en uno solo al compilar.

---

### `<xacro:macro>` -- Macros (funciones reutilizables)

Definen bloques de URDF que se pueden reusar con parametros distintos. Ideal para las dos ruedas (mismo diseno, distinta posicion):

```xml
<!-- Definicion de la macro -->
<xacro:macro name="wheel" params="prefix y_offset">
  <link name="${prefix}_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <xacro:inertial_cylinder mass="${wheel_mass}"
                              length="${wheel_width}" radius="${wheel_radius}"/>
  </link>

  <joint name="${prefix}_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="${prefix}_wheel_link"/>
    <origin xyz="0 ${y_offset} 0" rpy="${-pi/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</xacro:macro>

<!-- Uso: una llamada por rueda -->
<xacro:wheel prefix="left"  y_offset="${wheel_separation/2}"/>
<xacro:wheel prefix="right" y_offset="${-wheel_separation/2}"/>
```

---

### `<link>` -- Cuerpo rigido

Cada pieza fisica del robot. Tiene 3 secciones:

```xml
<link name="base_link">

  <!-- VISUAL: lo que se ve en RViz / Isaac Sim -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>          <!-- offset respecto al origen del link -->
    <geometry>
      <box size="0.30 0.25 0.10"/>              <!-- primitiva: box, cylinder, sphere -->
      <!-- o mesh de Fusion: -->
      <!-- <mesh filename="package://cargo_bot_description/meshes/visual/base_link.stl"
                  scale="0.001 0.001 0.001"/> -->
    </geometry>
    <material name="blue">                       <!-- color (solo RViz, Isaac usa sus propios) -->
      <color rgba="0.2 0.2 0.8 1.0"/>
    </material>
  </visual>

  <!-- COLLISION: lo que la fisica usa para detectar choques -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.30 0.25 0.10"/>              <!-- primitivas son mas rapidas que meshes -->
    </geometry>
  </collision>

  <!-- INERTIAL: masa + inercia (OBLIGATORIO para simulacion) -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>          <!-- centro de masa respecto al origen del link -->
    <mass value="2.5"/>
    <inertia ixx="0.005" ixy="0.0" ixz="0.0"
             iyy="0.007" iyz="0.0"
             izz="0.009"/>
  </inertial>
  <!-- o usar una macro: <xacro:inertial_box mass="2.5" x="0.30" y="0.25" z="0.10"/> -->

</link>
```

**Reglas:**
- Cada link DEBE tener las 3 secciones (visual, collision, inertial) para que funcione en simulacion
- Excepcion: links "dummy" como `base_footprint` pueden no tener visual/collision (solo sirven como frame de referencia)
- `<origin>` es relativo al origen del link, NO al mundo

---

### `<joint>` -- Conexion entre links

Define como se conectan dos links y como se mueven entre si:

```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>                    <!-- link padre -->
  <child link="left_wheel_link"/>               <!-- link hijo -->
  <origin xyz="0 0.13 0" rpy="${-pi/2} 0 0"/>  <!-- posicion del hijo respecto al padre -->
  <axis xyz="0 0 1"/>                           <!-- eje de rotacion (en frame del joint) -->
</joint>
```

**Tipos de joint:**

| Tipo | Movimiento | Uso en cargo_bot |
|------|-----------|-----------------|
| `fixed` | Ninguno, rigido | caster, lidar, imu, base_footprint→base_link |
| `continuous` | Rotacion infinita | ruedas motrices |
| `revolute` | Rotacion con limites | (no lo usamos) |
| `prismatic` | Traslacion lineal | (no lo usamos) |

**`<origin>`** -- la posicion y rotacion del frame del hijo respecto al padre:
- `xyz` = traslacion en metros (x adelante, y izquierda, z arriba)
- `rpy` = rotacion en radianes (roll, pitch, yaw)
- Ejemplo: `rpy="${-pi/2} 0 0"` rota -90 grados en X (para poner un cilindro horizontal como rueda)

**`<axis>`** -- el eje alrededor del cual gira el joint:
- `xyz="0 0 1"` = gira alrededor de Z del frame del joint
- Para ruedas: despues de rotar el frame con `rpy`, Z del joint queda apuntando al eje de la rueda

---

### `<material>` -- Colores (solo RViz)

```xml
<material name="blue">
  <color rgba="0.2 0.2 0.8 1.0"/>    <!-- R G B Alpha, valores 0.0 a 1.0 -->
</material>
```

Se pueden definir una vez y referenciar por nombre:
```xml
<!-- Definir arriba del todo -->
<material name="white">
  <color rgba="1.0 1.0 1.0 1.0"/>
</material>

<!-- Usar en cualquier link -->
<visual>
  <geometry>...</geometry>
  <material name="white"/>            <!-- referencia por nombre -->
</visual>
```

Isaac Sim ignora estos colores — usa sus propios materiales. Solo sirven para RViz.

---

### Constantes utiles de Xacro

```xml
<!-- pi ya viene definido en xacro, no hace falta declararlo -->
<origin rpy="${pi/2} 0 0"/>     <!-- 90 grados -->
<origin rpy="${-pi/2} 0 0"/>    <!-- -90 grados -->
<origin rpy="0 0 ${pi}"/>      <!-- 180 grados -->
```

---

### `<xacro:if>` y `<xacro:unless>` -- Condicionales

Sirven para tener variantes (ej: con o sin lidar):

```xml
<xacro:property name="use_lidar" value="true"/>

<xacro:if value="${use_lidar}">
  <!-- solo se incluye si use_lidar es true -->
  <xacro:include filename="sensors.xacro"/>
</xacro:if>
```

---

### Compilar Xacro a URDF

Xacro es un preprocesador. El resultado final es URDF puro. Para verificar:

```bash
# Compilar y ver el URDF generado
ros2 run xacro xacro cargo_bot.urdf.xacro

# Guardar a archivo (necesario para importar en Isaac Sim)
ros2 run xacro xacro cargo_bot.urdf.xacro > cargo_bot.urdf

# Verificar que el URDF es valido
check_urdf cargo_bot.urdf
```

---

### Ejemplo minimo completo (un cubo con una rueda)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="ejemplo">

  <xacro:property name="body_mass" value="1.0"/>
  <xacro:property name="body_size" value="0.2"/>
  <xacro:property name="wheel_r" value="0.04"/>
  <xacro:property name="wheel_w" value="0.02"/>

  <!-- Link 1: cuerpo -->
  <link name="base_link">
    <visual>
      <geometry><box size="${body_size} ${body_size} ${body_size}"/></geometry>
    </visual>
    <collision>
      <geometry><box size="${body_size} ${body_size} ${body_size}"/></geometry>
    </collision>
    <inertial>
      <mass value="${body_mass}"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Link 2: rueda -->
  <link name="wheel_link">
    <visual>
      <geometry><cylinder radius="${wheel_r}" length="${wheel_w}"/></geometry>
    </visual>
    <collision>
      <geometry><cylinder radius="${wheel_r}" length="${wheel_w}"/></geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint: conecta cuerpo con rueda -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 ${body_size/2 + wheel_w/2} 0" rpy="${-pi/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

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
