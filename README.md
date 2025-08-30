# ATUM-VRC-High-Stakes-Code

[![Rust](https://shields.io/badge/-Rust-3776AB?style=flat&logo=rust)](https://www.rust-lang.org)

This repository contains the code used for team **ATUM’s robots** for the V5RC 2024–2025 game, *Push Back*.

## Project Structure

Inside of the packages folder you will find

- **`atum/`** → Shared code used across all robots (e.g. control algorithms, sensors, utility functions).  
- **`pink/` / `white/`** → Individual robot projects. You must `cd` into one of these to build/upload for that specific robot.

## Set Up

1. **Install Git**  
Make sure you have [Git](https://git-scm.com/downloads) installed.

2. **Install Rust (via Rustup)**  

    Follow the instructions from [Rustup](https://www.rust-lang.org/tools/install).

    After installation, verify with:

    ```bash
    rustc --version
    cargo --version
    ```

3. **Clone this repository**

    ```bash
    git clone https://github.com/YOUR-ORG/ATUM-VRC-High-Stakes-Code.git
    cd ATUM-VRC-High-Stakes-Code
    ```

4. **Install Vexide tooling**

    Follow the instructions from [Vexide](https://github.com/vexide/vexide) to setup the rust work environment and cargo-v5 subcommand.

    This allows you to run `cargo v5 build` and `cargo v5 upload`
    Make sure to run these commands in release mode by adding `--release`

## Building and Uploading

1. Change directory

    ```bash
    cd packages/pink
    ```

2. Build the code

    ```bash
    cargo v5 build --release
    ```

3. Upload the code (will build the project as well)

    ```bash
    cargo v5 upload --release
    ```
