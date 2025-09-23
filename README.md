# Robot Kinematics Lecture Notes

[![GitHub](https://img.shields.io/badge/GitHub-Repository-blue?style=flat&logo=github)](https://github.com/haijunsu-osu/lec_robot_kinematics)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![PDF](https://img.shields.io/badge/PDF-Download-red?style=flat&logo=adobeacrobatreader)](https://github.com/haijunsu-osu/lec_robot_kinematics/raw/main/robot_kinematics.pdf)

## Overview

This repository contains comprehensive lecture notes and computational examples for **Robot Kinematics**, developed as educational material for robotics courses. The content closely follows the theoretical framework and examples presented in **J. Craig's "Introduction to Robotics: Mechanics and Control" (3rd Edition)**.

## 📚 Contents

### Lecture Notes (PDF)
- [**Lecture Notes**](https://github.com/haijunsu-osu/lec_robot_kinematics/raw/main/robot_kinematics.pdf) (opens in new tab)
  - Forward kinematics and coordinate transformations
  - Denavit-Hartenberg (DH) parameters
  - Inverse kinematics with analytical solutions
  - PUMA 560 robot analysis
  - Mathematical formulations and appendices

## 💻 Code Examples

The [`source_codes/`](source_codes/) directory contains implementations in multiple programming languages and platforms:

### Jupyter Notebooks (Python)
- **`forward_kinematics_examples.ipynb`** - Forward kinematics computations with DH parameters, transformation matrices, and coordinate frame analysis
  [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/haijunsu-osu/lec_robot_kinematics/blob/main/source_codes/forward_kinematics_examples.ipynb)
- **`puma_560_inverse_kinematics.ipynb`** - Comprehensive analytical inverse kinematics solution for PUMA 560 robot with all 8 possible configurations
  [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/haijunsu-osu/lec_robot_kinematics/blob/main/source_codes/puma_560_inverse_kinematics.ipynb)
- **`robot_kinematics_examples.ipynb`** - General robotics examples covering various kinematic concepts and computational methods
  [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/haijunsu-osu/lec_robot_kinematics/blob/main/source_codes/robot_kinematics_examples.ipynb)

### Mathematica Notebooks
- **`Inverse Kinematics of PUMA 560 (J Craig definition) Numerical.nb`** - Numerical solution methods for PUMA 560 inverse kinematics following Craig's formulation
- **`PUMA560 Robot (CraigBook) with Symbolic Derivation.nb`** - Symbolic mathematical derivation of PUMA 560 kinematics equations
- **`PUMA560 Robot (Verify JC book derivation).nb`** - Verification and validation of Craig's book derivations with symbolic computation

### MATLAB Scripts
- **`PUMA560Kinematics_FollowNotes_Answer.m`** - MATLAB implementation of PUMA 560 kinematics following the lecture notes with complete solutions


### LaTeX Source Files
All LaTeX source files are organized in the [`latex_source/`](latex_source/) directory:
- **Main document**: `robot_kinematics.tex`
- **Bibliography**: `references.bib` (IEEE format)
- **Section files**: Individual `.tex` files for each chapter
- **Images**: `imgs/` folder containing all figures and diagrams

## 🚀 Getting Started

### Prerequisites
- **LaTeX**: For compiling the lecture notes (MiKTeX, TeX Live, or similar)
- **Python 3.x**: For Jupyter notebooks (with NumPy, Matplotlib, SymPy)
- **Mathematica**: For .nb files (Wolfram Mathematica)
- **MATLAB**: For .m files (MATLAB or GNU Octave)

### Compiling LaTeX Notes
To regenerate the PDF from LaTeX source:

```bash
cd latex_source
pdflatex robot_kinematics.tex
bibtex robot_kinematics
pdflatex robot_kinematics.tex
pdflatex robot_kinematics.tex
```

### Running Code Examples
1. **Jupyter Notebooks**: 
   - **Local**: Open with Jupyter Lab/Notebook or VS Code
   - **Cloud**: Click the "Open in Colab" badges above to run directly in Google Colab (no installation required)
2. **Mathematica**: Open .nb files in Wolfram Mathematica
3. **MATLAB**: Run .m files in MATLAB environment

## 📖 Topics Covered

1. **Coordinate Systems and Transformations**
   - Homogeneous transformations
   - Rotation matrices
   - Euler angles

2. **Forward Kinematics**
   - Denavit-Hartenberg (DH) parameters
   - Link transformations
   - End-effector pose calculation

3. **Inverse Kinematics**
   - Analytical solutions
   - Multiple configurations
   - PUMA 560 case study

4. **Mathematical Foundations**
   - Trigonometric identities
   - Matrix operations
   - Kinematic equations

## 🎯 Learning Objectives

After studying these materials, students will be able to:
- Understand coordinate transformations in robotics
- Apply DH parameters for forward kinematics
- Solve inverse kinematics problems analytically
- Implement kinematic algorithms in multiple programming environments
- Analyze industrial robot configurations (PUMA 560)


## 📚 References

1. **Craig, J. J.** (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson/Prentice Hall.
2. **Lynch, K. M., & Park, F. C.** (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.
3. **McCarthy, J. M., & Soh, G. S.** (2011). *Geometric Design of Linkages*. Springer.
4. **Waldron, K. J., Kinzel, G. L., & Agrawal, S. K.** (2016). *Kinematics, Dynamics, and Design of Machinery* (3rd ed.). Wiley.

## 📄 License

This educational material is released under the MIT License. See [LICENSE](LICENSE) for details.

## 👥 Contributing

Contributions to improve the educational content are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Submit a pull request with clear descriptions

## 📧 Contact

For questions or suggestions regarding these lecture notes, please open an issue on GitHub.

---

*Developed for educational purposes in robotics engineering courses.*