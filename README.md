# SHG Mapping - ANC300 Ethernet

Windows laboratory-control application for two-dimensional SHG mapping with:

- attocube ANC300 Ethernet control for open-loop X/Y scanning;
- an MT controller dedicated to half-wave-plate rotation;
- C8855-01 PMT acquisition;
- hardware-free simulation and automated tests.

## First run

1. Install 64-bit Python with `tkinter`, or use a compatible Anaconda installation.
2. Copy `config.example.json` to `config.json` and enter local controller settings. `config.json` is intentionally ignored by Git because it can contain a plaintext ANC300 password and laboratory IP address.
3. Double-click `run_mapping.bat`.
4. For a dry run, actively enable **Simulation Mode** before connecting the simulated controllers.

The application starts disconnected and does not enable or restore outputs automatically.

## Real-hardware safety order

1. Connect and inspect the ANC300 identity and X/Y module state.
2. Physically confirm the scanner profile, approximately 4 K operation, and the 0-150 V rating.
3. Confirm the hardware profile in the application.
4. Enable offset outputs.
5. Set the current position as the session origin.
6. Begin with a supervised low-voltage, small-range scan.

The software hard-limits offsets to 0-150 V and voltage ramp increments to at most 1 V. Output enable and grounding verify measured output (`geto`) and require AC-IN/DC-IN to be off. Position values are calibration-based estimates.

See `README.txt` for the detailed Chinese operating instructions and `ANC300/Manuals/Manual_ANC300_v3.4.pdf` for the controller manual.

## Tests

```powershell
py -B -m unittest discover
```

The test suite uses local fakes and a fake TCP ANC300 server; it does not operate laboratory hardware.
