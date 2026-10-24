# wuji_tools

`wuji_tools` provides small device-independent command-line tools backed by the Wuji SDK.

## List Devices

Run the scanner after sourcing the workspace:

```bash
ros2 run wuji_tools wuji_list
```

The command sorts results by device type, serial number, and endpoint. It prints the SDK address unchanged:

```text
device_type: WujiHand2
serial_number: WH2J...
hand_side: left
endpoint: <sdk-address>
```

For Wuji Hand 2 and Wuji Glove scan results, `wuji_list` reads serial number index 3. `J` means left, and `K` means right. Short serial numbers, other markers, and unsupported device types remain `unknown`. The scanner doesn't connect to a device or normalize the SDK address.

Run `wuji_list --help` for usage. Unsupported arguments and SDK scan failures return a nonzero status and write an error to standard error.

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
