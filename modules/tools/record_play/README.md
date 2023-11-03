# Record and Play Tool

## Prerequisite

Run the following command from your JmcAuto root dir:

```bash
bash jmc_auto.sh build
source scripts/jmc_auto_base.sh
```

## Recorder

This tool records trajectory information from gateway into a csv file, the file
name is defined in filename_path.

```bash
python modules/tools/record_play/recorder_path.py
```

## Player

This tool reads information from a csv file and publishes planning trajectory in
the same format as real planning node.

Argument: Speed multiplier (default: 100)

```bash
python modules/tools/record_play/player_path.py [-s speed_multiplier]
```
