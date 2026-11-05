# Conveyor Sorting Environment (Tahap 1)

**Target Environment:**
- Isaac Sim 6.2.0
- Isaac Lab 3.0.0
- Python 3.12
- Physics Backend: PhysX

---

## 1. Menjalankan Milestone 1 (M1: Conveyor Spike)

Skrip M1 Spike menguji integrasi *conveyor belt* dan memvalidasi kriteria penerimaan **AT-2** (kecepatan belt sumbu X berada dalam selisih $\le 10\%$ dari kecepatan target $0.2\text{ m/s}$).

### Cara A: Windows (menggunakan `isaaclab.bat` di Command Prompt / PowerShell)

Buka terminal di root direktori IsaacLab:

**1. Menjalankan Uji AT-2 (Default Mode: `custom_drive` GPU-compatible):**
```cmd
.\isaaclab.bat -p source\conveyor_sorting\conveyor_sorting\scripts\spike_conveyor.py --headless --num_envs 1 --speed 0.2
```

**2. Menampilkan GUI / Viewport 3D Langsung:**
Di Isaac Lab 3.0, tampilan visual viewport dibuka dengan menambahkan flag `--viz kit`:
```cmd
.\isaaclab.bat -p source\conveyor_sorting\conveyor_sorting\scripts\spike_conveyor.py --viz kit --num_envs 1 --speed 0.2
```

**3. Uji Multi-Environment (`num_envs = 4`):**
```cmd
.\isaaclab.bat -p source\conveyor_sorting\conveyor_sorting\scripts\spike_conveyor.py --headless --num_envs 4 --speed 0.2
```

**4. Opsi Mode Alternatif:**
- `--mode custom_drive` (Default): Menggunakan tensor velocity drive bawaan Isaac Lab (100% kompatibel dengan GPU RTX 5060 Ti dan multi-env).
- `--mode surface_api`: Menggunakan `PhysxSurfaceVelocityAPI` murni (khusus CPU: tambahkan `--device cpu`).
- `--mode omnigraph`: Menggunakan Conveyor Belt Utility bawaan Isaac Sim.

---

### Cara B: Linux / uv (jika menggunakan environment uv)

**Single-Environment (`num_envs = 1`):**
```bash
uv run python source/conveyor_sorting/conveyor_sorting/scripts/spike_conveyor.py --headless --num_envs 1 --speed 0.2
```

**Multi-Environment (`num_envs = 4`):**
```bash
uv run python source/conveyor_sorting/conveyor_sorting/scripts/spike_conveyor.py --headless --num_envs 4 --speed 0.2
```

---

## 2. Pengujian Unit Logika Murni (Tanpa Simulator)
Logika penjadwalan spawn, pengecekan *clearance gap*, pengelolaan pool objek, dan pemantauan kondisi despawn dapat diuji di mesin mana pun tanpa dependensi Isaac Sim:

```bash
PYTHONPATH=source/conveyor_sorting python3 -m unittest source/conveyor_sorting/tests/test_pure_logic.py
```
