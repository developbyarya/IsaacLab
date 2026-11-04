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

**Single-Environment (`num_envs = 1`):**
```cmd
.\isaaclab.bat -p source\conveyor_sorting\conveyor_sorting\scripts\spike_conveyor.py --headless --num_envs 1 --speed 0.2
```

**Multi-Environment (`num_envs = 4`):**
```cmd
.\isaaclab.bat -p source\conveyor_sorting\conveyor_sorting\scripts\spike_conveyor.py --headless --num_envs 4 --speed 0.2
```

**Fallback Mode (Direct `PhysxSurfaceVelocityAPI`):**
```cmd
.\isaaclab.bat -p source\conveyor_sorting\conveyor_sorting\scripts\spike_conveyor.py --headless --num_envs 4 --mode surface_api
```

*(Catatan: Anda juga dapat menghapus flag `--headless` jika ingin melihat visualisasi 3D viewport di layar monitor).*

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
