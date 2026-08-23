# Vision Pipeline — Detecção de Campo, Obstáculos e Linhas

Implementação didática inspirada no artigo *"An Open Source Vision Pipeline
Approach for RoboCup Humanoid Soccer"* (Fiedler, Brandt, Gutsche, Vahl,
Hagge, Bestmann — Hamburg Bit-Bots). Reproduz a arquitetura **pipe-and-filter**
descrita no artigo: cada módulo é um filtro independente que recebe uma
imagem e produz uma saída estruturada, sem se preocupar com o que vem antes
ou depois dele no pipeline.

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
        (orquestra + debug image)
```

---

## Módulos

### `color_common.py`

Definição:

- **`ColorSpaceHSV`** — dataclass com os limites min/max de um espaço de cor
  HSV (H, S, V). Método `.mask(imagem)` retorna a máscara binária dos pixels
  dentro da faixa.
- **`ColorDetector`** — agrupa os 5 espaços de cor usados no pipeline:
  `field` (verde do campo), `marking` (branco das linhas), `red`, `blue`
  (marcadores de time) e `white` (traves). Expõe `field_mask()` e
  `marking_mask()` para os outros módulos.
- **`DEFAULT_COLOR_SPACES`** — valores de exemplo/hardcoded, usados apenas
  para teste antes da calibração real.
- **`load_color_spaces_from_csv(csv_path)`** — **é aqui que fica o
  `pd.read_csv("meus_valores_hsv.csv")`**. Lê um CSV com colunas
  `classe,h_min,h_max,s_min,s_max,v_min,v_max` e devolve o dicionário de
  `ColorSpaceHSV` prontos para montar um `ColorDetector` real.
- **`build_color_detector(color_spaces)`** — monta o `ColorDetector` a
  partir do dict (venha ele do CSV ou do default).

Corresponde ao módulo **Color Detector** do artigo (Seção 3.1): no artigo,
o campo usa uma lookup table (YAML) adaptativa e os marcadores usam HSV
configurável manualmente antes da partida — aqui simplificamos tudo para
HSV configurável via CSV, mas a interface (`ColorDetector`) é a mesma
independente da fonte dos valores.

### `field_boundary_detector.py`

Função: **`detect_field_boundary(image_bgr, color_detector, column_step,
search_from_top, kernel_size)`**

Encontra a borda do campo por **varredura em colunas** (scanline), não
pixel a pixel — a otimização de runtime central do artigo. Dois modos:

- `search_from_top=True`: varre cada coluna de cima para baixo até achar
  verde. Rápido; usado quando o robô olha para baixo (campo ocupa a maior
  parte da imagem).
- `search_from_top=False`: varre de baixo para cima até achar não-verde,
  com um kernel morfológico para não confundir linhas brancas internas com
  a borda externa. Mais lento; usado quando a cabeça do robô está
  inclinada para cima.

Depois calcula o **convex hull** dos pontos encontrados, eliminando os
"dentes" causados por obstáculos que tampam parte da borda real.

Retorna `(boundary_points, hull_mask)` — a lista de pontos é reaproveitada
por `obstacle_detector.py` e `line_detector.py`.

### `obstacle_detector.py`

Função: **`detect_obstacles(image_bgr, color_detector, field_boundary_points,
hull_mask, min_area)`**

Recebe a saída do módulo anterior. A lógica: a diferença entre o
`hull_mask` (campo "ideal", convexo) e a máscara da fronteira real
detectada é exatamente a região "roubada" por um obstáculo. Componentes
conexos dessa diferença, com área acima de `min_area`, viram candidatos.
Cada candidato é classificado pela **cor média** dentro da sua bounding
box: predominância de branco → trave; vermelho/azul → robô do time
correspondente.

Retorna uma lista de `Obstacle` (dataclass com `x, y, width, height,
color_class` e a propriedade `.bbox`).

### `line_detector.py`

Função: **`detect_line_points(image_bgr, color_detector,
field_boundary_points, n_samples, rng, previous_detections,
density_boost_radius)`**

Não detecta linhas geometricamente — segue a abordagem do artigo de
retornar **pontos** pertencentes a marcações, mais barato
computacionalmente e suficiente para localização. Amostra pixels
aleatórios abaixo do topo da fronteira do campo, com densidade maior:

- em linhas mais baixas da imagem (mais perto do robô);
- perto de onde houve detecção no frame anterior (`previous_detections`),
  simulando a continuidade citada no artigo.

Cada amostra só é aceita se cair dentro da `marking_mask`.

### `main_pipeline.py`

Orquestra os três módulos na ordem do diagrama do artigo (Fig. 2) e gera
uma imagem de debug (`debug_output.jpg`) similar à Fig. 1 do paper: pontos
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

O `pd.read_csv` roda dentro de `load_color_spaces_from_csv`, em
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

O artigo usa ROS como middleware e publica mensagens padronizadas (Seção
1 e Fig. 2, bloco "ROS message generation"). Para chegar lá a partir deste
código:

1. **Node de vision**: crie um nó (`vision_node.py`) que se inscreve no
   tópico da câmera (ex. `/image_raw`, tipo `sensor_msgs/Image`), converte
   para `numpy`/OpenCV via `cv_bridge`, e chama `main_pipeline.run(image)`
   a cada frame recebido.
2. **Publicação por módulo**: como no artigo, cada tipo de detecção vira
   uma mensagem própria — por exemplo:
   - `field_boundary_points` → mensagem custom `PointArray` ou
     `sensor_msgs/PointCloud`.
   - `obstacles` (lista de `Obstacle`) → mensagem custom tipo
     `ObstacleArray` (bounding box + classe de cor), similar ao que o
     artigo chama de mensagens padronizadas de obstáculo/robô.
   - `line_points` → outra `PointArray`/`PointCloud`, consumida
     futuramente pelo módulo de localização (o filtro de partículas que
     você já implementou é justamente o consumidor natural desses pontos:
     eles substituiriam as observações simuladas por
     `obter_observacao_camera()`).
3. **Parâmetros dinâmicos**: os valores HSV (hoje fixos via CSV) podem
   virar parâmetros ROS (`rosparam`/`dynamic_reconfigure` no ROS 1, ou
   parâmetros declarados + callback no ROS 2), permitindo recalibrar em
   tempo real durante testes, como o artigo faz.
4. **Debug**: `draw_debug_image()` em `main_pipeline.py` já gera a imagem
   de debug — no ROS isso vira só mais uma publicação, num tópico
   `/vision/debug_image`, ativável por parâmetro (o artigo torna esse
   passo opcional por custo de performance).
5. **Separação de threads/nós**: o artigo roda detecção convencional e
   FCNN da bola em paralelo (duas threads dentro do mesmo nó, para evitar
   overhead de mensageria entre nós). Ao integrar no ROS 2, considere o
   mesmo: um nó só, com as chamadas aos três módulos + FCNN rodando de
   forma assíncrona, publicando ao final.

## Integração futura com redes neurais (FCNN da bola)

O artigo usa uma FCNN só para a bola (os outros três — fronteira,
obstáculo, linha — são heurísticos, como implementado aqui). 

Passos para integrar:

1. **Módulo separado**: crie `ball_fcnn_detector.py`, paralelo aos três
   já existentes, seguindo a mesma convenção de assinatura
   (`detect_ball(image_bgr, model) -> BallCandidate`).
2. **Pré-processamento**: redimensionar a imagem para o tamanho de entrada
   do modelo (o "FCNN Handler" do artigo).
3. **Extração de candidatos**: a saída da FCNN é um heatmap
   pixel-a-pixel (ativação 0–1); é preciso extrair o pico/blob mais forte
   — o artigo faz isso em C++ por performance, mas um
   `cv2.connectedComponentsWithStats` sobre o heatmap binarizado (como já
   usado em `obstacle_detector.py`) resolve numa primeira versão em
   Python puro.
4. **Classe unificadora (`Candidate Finder`)**: se no futuro você quiser
   generalizar `Obstacle` e o candidato da bola sob uma interface comum
   (o artigo propõe as classes `Candidate`/`CandidateFinder` para isso),
   vale extrair uma classe base com múltiplas representações (canto+dim,
   centro+raio) e um método `contains(ponto)`.
5. **Uso do resultado do filtro de partículas**: os `line_points` e a
   fronteira do campo, gerados aqui, são exatamente as observações que
   alimentam a localização Monte Carlo já implementada — feche o
   ciclo trocando o simulador (`obter_observacao_camera()`) por chamadas
   reais a `detect_line_points()`/`detect_field_boundary()` na imagem ao
   vivo.

---

## Arquivos

| Arquivo | Função principal | Depende de |
|---|---|---|
| `color_common.py` | `ColorDetector`, `load_color_spaces_from_csv` | pandas, opencv |
| `field_boundary_detector.py` | `detect_field_boundary` | `color_common.py` |
| `obstacle_detector.py` | `detect_obstacles` | `color_common.py` |
| `line_detector.py` | `detect_line_points` | `color_common.py` |
| `main_pipeline.py` | `run`, `draw_debug_image` | os quatro acima |
