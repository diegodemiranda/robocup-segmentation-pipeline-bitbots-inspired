# Pipeline de Segmentação / Segmentation Pipeline

[🇧🇷 Versão em Português](#-pipeline-de-segmentação--detecção-de-campo-obstáculos-e-linhas) | [🇺🇸 English Version](#-segmentation-pipeline--field-boundary-obstacle-and-line-detection)

---

## 🇧🇷 Pipeline de Segmentação — Detecção de Campo, Obstáculos e Linhas

Implementação didática inspirada no artigo *"An Open Source Vision Pipeline
Approach for RoboCup Humanoid Soccer"* (Fiedler, Brandt, Gutsche, Vahl,
Hagge, Bestmann — Hamburg Bit-Bots). Reproduz a arquitetura
**pipe-and-filter** descrita no artigo: cada módulo é um filtro independente
que recebe uma imagem e produz uma saída estruturada, sem se preocupar com
o que vem antes ou depois dele no pipeline.

```
              ┌────────────────────┐
   imagem ──▶ │  color_common.py   │  (compartilhado por todos os módulos)
              └─────────┬──────────┘
                         │
        ┌────────────────┼───────────────┐
        ▼                ▼               ▼
field_boundary_    obstacle_        line_detector.py
detector.py         detector.py
        │                │               │
        └───────┬────────┴───────────────┘
                 ▼
          main_pipeline.py
        (orquestra + imagem de debug)
```

---

## Módulos

### `color_common.py`

Definição:

- **`ColorSpaceHSV`** — dataclass com os limites mínimo/máximo de um espaço
  de cor HSV (H, S, V). O método `.mask(imagem)` retorna a máscara binária
  dos pixels dentro da faixa.
- **`ColorDetector`** — agrupa os 5 espaços de cor usados no pipeline:
  `field` (verde do campo), `marking` (branco das linhas), `red`, `blue`
  (marcadores de time) e `white` (traves). Expõe `field_mask()` e
  `marking_mask()` para os demais módulos.
- **`DEFAULT_COLOR_SPACES`** — valores de exemplo/hardcoded, usados apenas
  para teste antes da calibração real.
- **`load_color_spaces_from_csv(csv_path)`** — é aqui que fica o
  `pd.read_csv("meus_valores_hsv.csv")`. Lê um CSV com colunas
  `classe,h_min,h_max,s_min,s_max,v_min,v_max` e devolve o dicionário de
  `ColorSpaceHSV` prontos para montar um `ColorDetector` real.
- **`build_color_detector(color_spaces)`** — monta o `ColorDetector` a
  partir do dicionário (venha ele do CSV ou do default).

Corresponde ao módulo **Color Detector** do artigo (Seção 3.1): no texto,
o campo usa uma lookup table (YAML) adaptativa e os marcadores usam HSV
configurável manualmente antes da partida. Aqui simplificamos tudo para HSV
configurável via CSV, mas a interface (`ColorDetector`) é a mesma,
independente da fonte dos valores.

### `field_boundary_detector.py`

Função: **`detect_field_boundary(image_bgr, color_detector, column_step,
search_from_top, kernel_size)`**

Encontra a borda do campo por **varredura em colunas** (scanline), não por
pixel a pixel — a otimização de runtime central do artigo. Dois modos:

- `search_from_top=True`: varre cada coluna de cima para baixo até encontrar
  verde. É rápido; é usado quando o robô olha para baixo (o campo ocupa a
  maior parte da imagem).
- `search_from_top=False`: varre de baixo para cima até encontrar não-verde,
  com um kernel morfológico para não confundir linhas brancas internas com a
  borda externa. É mais lento; é usado quando a cabeça do robô está inclinada
  para cima.

Depois, calcula o **convex hull** dos pontos encontrados, eliminando os
"dentes" causados por obstáculos que tapam parte da borda real.

Retorna `(boundary_points, hull_mask)`: a lista de pontos é reaproveitada por
`obstacle_detector.py` e `line_detector.py`.

### `obstacle_detector.py`

Função: **`detect_obstacles(image_bgr, color_detector, field_boundary_points,
hull_mask, min_area)`**

Recebe a saída do módulo anterior. A lógica é a seguinte: a diferença entre o
`hull_mask` (campo "ideal", convexo) e a máscara da fronteira real detectada
é exatamente a região "roubada" por um obstáculo. Componentes conexos dessa
diferença, com área acima de `min_area`, viram candidatos. Cada candidato é
classificado pela **cor média** dentro da sua bounding box: predominância de
branco → trave; vermelho/azul → robô do time correspondente.

Retorna uma lista de `Obstacle` (dataclass com `x, y, width, height,
color_class` e a propriedade `.bbox`).

### `line_detector.py`

Função: **`detect_line_points(image_bgr, color_detector,
field_boundary_points, n_samples, rng, previous_detections,
density_boost_radius)`**

Não detecta linhas geometricamente — segue a abordagem do artigo de
retornar **pontos** pertencentes às marcações, o que é mais barato
computacionalmente e suficiente para localização. A amostra seleciona pixels
aleatórios abaixo do topo da fronteira do campo, com densidade maior:

- em linhas mais baixas da imagem (mais próximas do robô);
- perto de onde houve detecção no frame anterior (`previous_detections`),
  simulando a continuidade citada no artigo.

Cada amostra só é aceita se cair dentro da `marking_mask`.

### `main_pipeline.py`

Orquestra os três módulos na ordem do diagrama do artigo (Fig. 2) e gera uma
imagem de debug (`debug_output.jpg`) semelhante à Fig. 1 do paper: pontos
vermelhos para fronteira/linhas, retângulos coloridos para obstáculos.

---

## Como trocar os valores HSV de exemplo pelos reais

Em `main_pipeline.py`:

```python
# Teste (valores de exemplo, hardcoded):
color_spaces = DEFAULT_COLOR_SPACES

# Produção (valores calibrados na câmera real):
color_spaces = load_color_spaces_from_csv("meus_valores_hsv.csv")
```

O `pd.read_csv` é executado dentro de `load_color_spaces_from_csv`, em
`color_common.py`. O CSV precisa ter uma linha por classe:

```csv
classe,h_min,h_max,s_min,s_max,v_min,v_max
field,35,85,50,255,30,255
marking,0,180,0,60,180,255
red,0,10,100,255,80,255
blue,100,130,100,255,80,255
white,0,180,0,60,180,255
```

---

## Integração futura com ROS

O artigo usa ROS como middleware e publishes mensagens padronizadas (Seção 1 e
Fig. 2, bloco "ROS message generation"). Para chegar até lá a partir deste
código:

1. **Node de visão**: crie um nó (`vision_node.py`) que se inscreve no tópico
   da câmera (por exemplo, `/image_raw`, tipo `sensor_msgs/Image`), converte
   para `numpy`/OpenCV via `cv_bridge` e chama `main_pipeline.run(image)` a
   cada frame recebido.
2. **Publicação por módulo**: como no artigo, cada tipo de detecção vira uma
   mensagem própria — por exemplo:
   - `field_boundary_points` → mensagem custom `PointArray` ou
     `sensor_msgs/PointCloud`.
   - `obstacles` (lista de `Obstacle`) → mensagem custom do tipo
     `ObstacleArray` (bounding box + classe de cor), similar ao que o artigo
     chama de mensagens padronizadas de obstáculo/robô.
   - `line_points` → outra `PointArray`/`PointCloud`, consumida futuramente
     pelo módulo de localização (o filtro de partículas que você já
     implementou é justamente o consumidor natural desses pontos: eles
     substituiriam as observações simuladas por `obter_observacao_camera()`).
3. **Parâmetros dinâmicos**: os valores HSV (hoje fixos via CSV) podem virar
   parâmetros ROS (`rosparam`/`dynamic_reconfigure` no ROS 1, ou parâmetros
   declarados + callback no ROS 2), permitindo recalibrar em tempo real
   durante testes, como o artigo faz.
4. **Debug**: `draw_debug_image()` in `main_pipeline.py` já gera a imagem de
   debug — no ROS isso vira apenas mais uma publicação em um tópico
   `/vision/debug_image`, ativável por parâmetro (o artigo torna esse passo
   opcional por custo de performance).
5. **Separação de threads/nós**: o artigo roda a detecção unconventional e a
   FCNN da bola em paralelo (duas threads dentro do mesmo nó, para evitar
   overhead de mensageria entre nós). Ao integrar no ROS 2, considere o mesmo:
   u nó único, com as chamadas aos três módulos + FCNN rodando de forma
   assíncrona, publicando ao final.

## Integração futura com redes neurais (FCNN da bola)

O artigo usa uma FCNN apenas para a bola (os outros três — fronteira,
obstáculo e linha — são heurísticos, como implementado aqui).

Passos para integrar:

1. **Módulo separado**: crie `ball_fcnn_detector.py`, paralelo aos três já
   existentes, seguindo a mesma convenção de assinatura
   (`detect_ball(image_bgr, model) -> BallCandidate`).
2. **Pré-processamento**: redimensionar a imagem para o tamanho de entrada do
   modelo (o "FCNN Handler" do artigo).
3. **Extração de candidatos**: a saída da FCNN é um heatmap pixel a pixel
   (ativação 0–1); é preciso extrair o pico/blob mais forte — o artigo faz
   isso em C++ por performance, mas um
   `cv2.connectedComponentsWithStats` sobre o heatmap binarizado (como já
   usado em `obstacle_detector.py`) resolve uma primeira versão em Python
   puro.
4. **Classe unificadora (`Candidate Finder`)**: se no futuro você quiser
   generalizar `Obstacle` e o candidato da bola sob uma interface comum (o
   artigo prope as classes `Candidate`/`CandidateFinder` para isso), vale
   extrair uma classe base com múltiplas representações (canto + dimensão,
   centro + raio) e um método `contains(ponto)`.
5. **Uso do resultado do filtro de partículas**: os `line_points` e a fronteira
   do campo, gerados aqui, são exatamente as observações que alimentam a
   localização Monte Carlo já implementada — feche o ciclo trocando o
   simulador (`obter_observacao_camera()`) por chamadas reais a
   `detect_line_points()`/`detect_field_boundary()` na imagem ao vivo.

---

## Arquivos

| Arquivo | Função principal | Depende de |
|---|---|---|
| `color_common.py` | `ColorDetector`, `load_color_spaces_from_csv` | pandas, opencv |
| `field_boundary_detector.py` | `detect_field_boundary` | `color_common.py` |
| `obstacle_detector.py` | `detect_obstacles` | `color_common.py` |
| `line_detector.py` | `detect_line_points` | `color_common.py` |
| `main_pipeline.py` | `run`, `draw_debug_image` | os quatro acima |

---

## 🇺🇸 Segmentation Pipeline — Field Boundary, Obstacle, and Line Detection

A modular computer vision pipeline for RoboCup humanoid soccer, implementing classical image processing techniques for field boundary detection, obstacle classification, and field marking point detection using HSV color segmentation and OpenCV.

This project is inspired by *An Open Source Vision Pipeline Approach for RoboCup Humanoid Soccer* by Fiedler, Brandt, Gutsche, Vahl, Hagge, and Bestmann from the Hamburg Bit-Bots team. It follows the **pipe-and-filter architecture**, in which each module performs an independent processing task and produces structured outputs for subsequent components.

The implementation is intended for experimentation, algorithm development, and integration into larger robotic perception systems.

### Related Projects

#### RoboCup Vision Pipeline

**Repository:** [robocup-vision-pipeline](https://github.com/diegodemiranda/robocup-vision-pipeline)

For a broader computer vision system integrating neural object detection and robotics middleware, see [RoboCup Vision Pipeline](https://github.com/diegodemiranda/robocup-vision-pipeline).

The two projects address complementary aspects of visual perception for RoboCup humanoid soccer:

- **Segmentation Pipeline (this repository):** focuses on classical image processing algorithms, including HSV color segmentation, field boundary detection, obstacle classification, and field marking point sampling.
- **RoboCup Vision Pipeline:** focuses on integrating YOLO-based object detection, detection validation, image processing, and ROS 2 communication within a Webots-based robotics workflow.

This repository can be explored and tested independently of the broader vision system. The projects are maintained separately to preserve the modularity and reusability of the segmentation algorithms.

The existence of the related project does not imply a direct software dependency: integration depends on the interfaces and data structures used by each component.

### Architecture

The pipeline uses a modular pipe-and-filter design. The shared color processing module provides HSV masks to the individual detectors, which can then be orchestrated by the main pipeline.

```text
                  ┌────────────────────┐
      Image ────▶ │  color_common.py   │
                  └─────────┬──────────┘
                            │
             ┌──────────────┼──────────────┐
             ▼              ▼              ▼
      field_boundary_  obstacle_      line_detector.py
      detector.py      detector.py
             │              │              │
             └──────────────┴──────────────┘
                            │
                            ▼
                     main_pipeline.py
                  Orchestration and debug
```

### Features

- HSV-based color segmentation with configurable color ranges.
- Field boundary detection using column-wise scanning.
- Convex hull computation to estimate the outer field boundary.
- Obstacle candidate extraction using the difference between the estimated field region and its detected boundary.
- Obstacle classification based on average color.
- Field marking point sampling using a binary marking mask.
- Optional sampling bias toward lower image regions and previously detected points.
- Debug image generation for visual inspection of detections.
- CSV-based configuration of HSV color ranges.

### Modules

#### `color_common.py`

Defines the shared color representation and segmentation utilities.

**`ColorSpaceHSV`**

A dataclass containing the minimum and maximum HSV values for a color range. Its `mask(image)` method returns a binary mask identifying pixels within the configured range.

**`ColorDetector`**

Groups the five color spaces used by the pipeline:

- `field`: green field surface.
- `marking`: white field markings.
- `red`: red team markers.
- `blue`: blue team markers.
- `white`: white goal structures.

It exposes methods such as `field_mask()` and `marking_mask()` for use by the detection modules.

**`DEFAULT_COLOR_SPACES`**

Provides example HSV values for initial testing. These values are placeholders and should be calibrated for the actual camera, lighting, and environment.

**`load_color_spaces_from_csv(csv_path)`**

Loads HSV ranges from a CSV file and converts them into `ColorSpaceHSV` objects.

The expected CSV columns are:

`classe,h_min,h_max,s_min,s_max,v_min,v_max`

**`build_color_detector(color_spaces)`**

Constructs a `ColorDetector` from a dictionary of HSV color ranges, regardless of whether the values come from the defaults or a CSV file.

This module corresponds conceptually to the Color Detector component described in Section 3.1 of the referenced paper. Unlike the paper's adaptive lookup-table approach for field color detection, this implementation uses configurable HSV ranges for all supported color classes.

#### `field_boundary_detector.py`

Main function:

```python
detect_field_boundary(
    image_bgr,
    color_detector,
    column_step,
    search_from_top,
    kernel_size
)
```

Detects the field boundary using column-wise scanning rather than evaluating every pixel independently.

Two scanning modes are available:

- **`search_from_top=True`:** scans each column from top to bottom until a green field pixel is found. This mode is intended for views in which the field occupies most of the image.
- **`search_from_top=False`:** scans from bottom to top, using a morphological kernel to reduce confusion between internal white markings and the external field boundary. This mode is intended for camera views with a more upward-facing orientation.

The detector then computes a convex hull from the detected points to reduce irregularities caused by obstacles occluding the field boundary.

Returns:

`(boundary_points, hull_mask)`

The detected points and hull mask are subsequently used by the obstacle and line detection modules.

#### `obstacle_detector.py`

Main function:

```python
detect_obstacles(
    image_bgr,
    color_detector,
    field_boundary_points,
    hull_mask,
    min_area
)
```

Identifies candidate obstacles using the difference between the convex hull representing the estimated field region and the detected field boundary.

Connected components in the resulting difference region are filtered by their area. Candidates exceeding `min_area` are classified according to the average color inside their bounding boxes.

The current implementation uses color predominance to distinguish candidate classes:

- White: goal structure candidate.
- Red: red team robot candidate.
- Blue: blue team robot candidate.

Returns a list of `Obstacle` objects.

Each `Obstacle` contains:

- `x`
- `y`
- `width`
- `height`
- `color_class`

The `.bbox` property provides the bounding-box representation.

**Note:** these are heuristic classifications based on color and geometry, not semantic classifications produced by a trained neural network.

#### `line_detector.py`

Main function:

```python
detect_line_points(
    image_bgr,
    color_detector,
    field_boundary_points,
    n_samples,
    rng,
    previous_detections,
    density_boost_radius
)
```

Rather than extracting complete line geometries, this module samples image points that belong to white field markings.

This design follows the point-based approach described in the reference paper, in which sampled marking points can provide observations for robot localization without requiring full geometric reconstruction of every line.

Sampling is biased toward:

- Lower image regions, where field markings are generally closer to the robot.
- Regions near detections from the previous frame, approximating temporal continuity.

A sampled point is accepted only if it falls within the marking mask.

The resulting points can be used by downstream localization algorithms, provided the expected coordinate conventions and data structures are compatible.

#### `main_pipeline.py`

Coordinates the field boundary, obstacle, and line detection modules.

The main pipeline combines their outputs and generates a debug image named `debug_output.jpg`, providing a visual representation of the detected field boundary, sampled marking points, and obstacle bounding boxes.

This module serves as the entry point for running the segmentation pipeline, subject to the interface exposed by its implementation.

### Configuration

#### Using default HSV values

The pipeline includes example HSV values for initial testing:

```python
color_spaces = DEFAULT_COLOR_SPACES
```

These values should not be assumed to work reliably under different lighting conditions, cameras, or simulation environments.

#### Loading calibrated HSV values from CSV

For calibrated color ranges, load the values from a CSV file:

```python
color_spaces = load_color_spaces_from_csv("my_hsv_values.csv")
```

The CSV file must contain one row per color class and the following columns:

```csv
classe,h_min,h_max,s_min,s_max,v_min,v_max
field,35,85,50,255,30,255
red,0,10,100,255,80,255
blue,100,130,100,255,80,255
white,0,180,0,60,180,255
```

The values above are examples only. They must be calibrated for the actual image source and environment.

The CSV loading operation is implemented in `load_color_spaces_from_csv()` in `color_common.py`.

### Installation

The implementation uses Python, OpenCV, NumPy, and pandas.

Install the dependencies in your preferred Python environment:

```bash
python -m pip install numpy opencv-python pandas
```

Clone the repository:

```bash
git clone https://github.com/diegodemiranda/robocup-segmentation-pipeline-bitbots-inspired.git
cd robocup-segmentation-pipeline-bitbots-inspired
```

The commands above assume that the repository is publicly available at the specified URL and that its files are located at the repository root.

### Running the Pipeline

The main entry point is `main_pipeline.py`.

```bash
python main_pipeline.py
```

This command assumes that the script's image input and configuration are already set up as expected by the implementation. Configure the input image or camera source and HSV values before running it in a different environment.

If the pipeline completes successfully, the current implementation generates a debug image named `debug_output.jpg`.

The precise runtime arguments and input configuration depend on the code in `main_pipeline.py`.

### ROS Integration

This repository currently describes a standalone image-processing pipeline. Its modules can serve as building blocks for a ROS 2 perception node, but ROS integration requires an additional adapter layer.

A possible integration architecture is:

1. Subscribe to a camera image topic, such as `/image_raw`, using `sensor_msgs/Image`.
2. Convert the incoming image into an OpenCV-compatible representation using `cv_bridge`.
3. Call the segmentation pipeline on each incoming frame.
4. Publish the resulting field boundary points, obstacle candidates, and marking points using suitable ROS 2 message types.
5. Optionally publish the debug image on a topic such as `/vision/debug_image`.

The output data structures must be mapped to the chosen ROS 2 message definitions. Custom messages may be required for obstacle bounding boxes and arrays of image points.

The detected marking points and field boundary may also be useful to a localization system, provided that the image observations are converted into the coordinate representation expected by that system.

These steps describe a possible integration path, not functionality that is necessarily implemented in this repository.

### Neural Network Integration

The referenced paper uses a fully convolutional neural network (FCNN) for ball detection alongside conventional image-processing modules.

A similar extension could introduce a dedicated detector, such as `ball_fcnn_detector.py`, while keeping the existing field boundary, obstacle, and line detection modules independent.

A potential implementation would:

1. Preprocess the input image according to the network's input requirements.
2. Run inference to obtain a heatmap or other model output.
3. Extract one or more ball candidates from the model output.
4. Convert the resulting detections into a representation compatible with the rest of the perception system.

The details depend on the selected model and its output format. Neural network ball detection is a potential extension and should not be considered an existing feature unless it has been implemented separately.

### Project Structure

The main modules and their responsibilities are summarized below.

| File | Responsibility | Main dependencies |
|---|---|---|
| `color_common.py` | HSV color spaces, masks, and color detector construction | NumPy, OpenCV, pandas |
| `field_boundary_detector.py` | Field boundary detection and convex hull computation | `color_common.py`, OpenCV |
| `obstacle_detector.py` | Obstacle candidate extraction and color-based classification | `color_common.py`, OpenCV |
| `line_detector.py` | Sampling points belonging to field markings | `color_common.py`, NumPy, OpenCV |
| `main_pipeline.py` | Pipeline orchestration and debug image generation | The modules above |

### Limitations

The current approach relies on classical image processing and heuristic rules. Its performance may vary with:

- Changes in lighting and shadows.
- Camera characteristics and color calibration.
- Occlusions and objects near the field boundary.
- Similarities between the colors of robots, field markings, and other objects.
- Camera orientation and the proportion of the image occupied by the field.

HSV thresholds should be calibrated for the target environment. The obstacle detector's color-based classification should not be interpreted as equivalent to learned object recognition.

The pipeline also does not, by itself, establish real-world field coordinates from image coordinates. This requires a compatible geometric projection or localization component.

### Reference

Fiedler, Brandt, Gutsche, Vahl, Hagge, and Bestmann. *An Open Source Vision Pipeline Approach for RoboCup Humanoid Soccer*. Hamburg Bit-Bots.

The implementation is inspired by the modular architecture and selected processing ideas described in this work. It should not be interpreted as a complete reproduction of the original system.

### License

Add a `LICENSE` file to this repository and specify the license under which the implementation is distributed.

Before choosing a license, verify the licensing terms applicable to the original reference material and any third-party code incorporated into this project.

### Contributing

Contributions that improve modularity, calibration, testing, performance, or integration with robotics systems are welcome.

When submitting changes, please:

1. Keep detection modules independent where practical.
2. Document changes to function signatures and output data structures.
3. Include tests or reproducible examples for new behavior.
4. Document any assumptions about image input, color calibration, or camera orientation.

### Acknowledgements

This project is inspired by the work of the Hamburg Bit-Bots team on modular vision pipelines for RoboCup humanoid soccer.

The related [RoboCup Vision Pipeline](https://github.com/diegodemiranda/robocup-vision-pipeline) explores a broader integration of neural object detection and robotics middleware.
