<div align="center">

<h1>TreeLoc++: Robust 6-DoF LiDAR Localization in Forests</h1>

Official repository for **TreeLoc++**.

<a href="https://arxiv.org/abs/2603.03695" target="_blank">Paper</a> |
<a href="https://github.com/minwoo0611/TreeLoc" target="_blank">TreeLoc</a>

</div>

### Recent Updates
- [2026/06/25] Initial TreeLoc++ branch with intra-session and inter-session C++ code.

### Contributions
- **TreeLoc++ performs forest localization on compact Digital Forest Inventories (DFIs)**, representing tree stems with geometric attributes rather than dense point-cloud maps.
- **TreeLoc++ improves retrieval in structurally ambiguous forests** by combining TDH retrieval with pairwise tree-layout context, triangle verification, DBH consistency, and yaw-consistent inlier selection.
- **TreeLoc++ estimates full 6-DoF corrections from tree geometry** by jointly refining height, roll, and pitch after geometric matching.
- **TreeLoc++ supports long-term and multi-session evaluation** across repeated forest traversals using the same tree-level map interface.

### Data Source and Tree Extraction

TreeLoc++ consumes tree-level observations extracted before evaluation. The bundled evaluation data covers Wild-Places Venman sequences and Oxford Forest Evo/Stein sequences; following TreeLoc, tree observations can be produced with [RealtimeTrees](https://github.com/ori-drs/realtime_trees) from the corresponding forest LiDAR recordings.

The bundled processed datasets correspond to:

- `Wild_V02` and `Wild_V03`: Wild-Places Venman sequences. The raw dataset is available from the [Wild-Places project page](https://csiro-robotics.github.io/Wild-Places/) and [CSIRO Data Access Portal](https://data.csiro.au/collection/csiro%3A56372).
- `Oxford_Evo` and `Oxford_Stein`: Oxford Forest sequences. The raw recordings are available from the [Oxford Forest Place Recognition Dataset](https://dynamic.robots.ox.ac.uk/datasets/oxford-forest/).

The evaluator expects a processed dataset root:

```text
dataset_root/
├── trajectory.txt
├── TreeManagerState_0.csv
├── TreeManagerState_1.csv
└── ...
```

`trajectory.txt` uses:

```text
timestamp x y z qx qy qz qw
```

Each `TreeManagerState_<idx>.csv` must include:

- `axis_00` ... `axis_22`
- `location_x`, `location_y`, `location_z`
- `dbh` or `dbh_approximation`

Optional columns used when available:

- `reconstructed`
- `number_clusters`
- `score` or `scores`

### Prerequisites

TreeLoc++ is a C++17 CMake project.

- CMake >= 3.16
- C++17 compiler
- Eigen3

Ubuntu:

```bash
sudo apt update
sudo apt install build-essential cmake libeigen3-dev
```

### Build

```bash
cmake -S . -B build
cmake --build build -j
```

This builds:

- `treelocpp_intra`
- `treelocpp_inter`

### Usage

Intra-session:

```bash
./build/treelocpp_intra config/default.yaml
```

Inter-session:

```bash
./build/treelocpp_inter config/inter.yaml
```

The publish tree keeps the four processed evaluation datasets under `data/`:

```text
data/Wild_V02/
data/Wild_V03/
data/Oxford_Evo/
data/Oxford_Stein/
```

Run intra-session evaluation:

```bash
./build/treelocpp_intra config/full_v02.yaml
./build/treelocpp_intra config/full_v03.yaml
./build/treelocpp_intra config/full_oxford_evo.yaml
./build/treelocpp_intra config/full_oxford_stein.yaml
```

Run inter-session evaluation:

```bash
./build/treelocpp_inter config/inter_v02_v03.yaml
```

`config/inter_v02_v03.yaml` uses the Wild_V/Wild_K test-region family selected by `test_region_family` and a 5 m ground-truth radius.

### Configuration

Main parameters are grouped by role in:

- `config/default.yaml` for intra-session evaluation on `data/Wild_V02`
- `config/inter.yaml` for inter-session evaluation on `data/Wild_V03` vs `data/Wild_V02`
- `config/full_v02.yaml` for full Wild_V02 intra-session evaluation
- `config/full_v03.yaml` for full Wild_V03 intra-session evaluation
- `config/full_oxford_evo.yaml` for full Oxford_Evo intra-session evaluation
- `config/full_oxford_stein.yaml` for full Oxford_Stein intra-session evaluation
- `config/inter_v02_v03.yaml` for full Wild_V03-to-Wild_V02 inter-session evaluation

The YAML files use sections such as `dataset`, `evaluation`, `retrieval`, `tree_selection`, `tdh`, `triangle_descriptor`, and `pose_refinement`; each variable has an inline comment describing its role. The public configuration keeps method-level settings exposed and leaves tree-axis alignment as the default TreeLoc++ descriptor-frame construction step. Legacy flat keys are still accepted by the parser for compatibility.

### TODO

- Multi-session Dataset (Evo25) upload
- Remaining dataset upload
- Multi-session graph optimization

### Acknowledgement

TreeLoc++ builds on [TreeLoc](https://arxiv.org/abs/2602.01501), tree-level representations extracted with [RealtimeTrees](https://github.com/ori-drs/realtime_trees), and forest LiDAR recordings from the [Wild-Places](https://csiro-robotics.github.io/Wild-Places/) and [Oxford Forest Place Recognition](https://dynamic.robots.ox.ac.uk/datasets/oxford-forest/) datasets.
