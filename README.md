<h1 align="center">
Unified Multi-Rate Model Predictive Control for a Jet-Powered Humanoid Robot
</h1>

<div align="center">
Davide Gorbani , Giuseppe L'Erario, Hosameldin Awadalla Omer Mohamed, Daniele Pucci
</div>
<br>

<div align="center">
📅 Accepted for publication at the <b>2025 IEEE-RAS International Conference on Humanoid Robots</b> (Humanoids), Seoul, South Korea 🤖
</div>
<br>

<div align="center">
   <a href="https://arxiv.org/abs/2505.16478"><b>📚 arXiv</b></a> &nbsp;&nbsp;&nbsp;
    <a href="#Installation"><b>🔧 Installation</b></a> &nbsp;&nbsp;&nbsp;
    <a href="#Usage"><b>🔧 Usage</b></a>
</div>
<be>


https://github.com/user-attachments/assets/4ad12276-6a8e-4b0a-91a7-b82744aea77c


## Installation

Clone this repo:
```sh
git clone https://github.com/ami-iit/paper_gorbani_2025_humanoids_multi-rate-mpc-ironcub
```

Create the `conda` environment:
```sh
cd paper_gorbani_2025_humanoids_multi-rate-mpc-ironcub
conda env create -f environment.yml
```

Install the repo [ironcub-models](https://github.com/ami-iit/ironcub-models) and then install this repo:
```sh
mkdir build
cd build
ccmake ..
make install
```

Then, to source the `setup.sh` file, run:
```sh
source <path/to/install/folder>/share/setup.sh
```

## Usage

To run a simulation, open a terminal, navigate to the root folder of this repo and run:
```sh
python src/variable_sampling_mpc.py
```

## Maintainers

<table>
  <tr>
    <td align="center">
      <a href="https://github.com/davidegorbani">
        <img src="https://github.com/davidegorbani.png" width="80" alt="Davide Gorbani"><br>
        Davide Gorbani
      </a>
    </td>
  </tr>
</table>


