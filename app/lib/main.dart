import 'dart:async';
import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:google_fonts/google_fonts.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle(
    statusBarColor: Color(0xFF151515),
    systemNavigationBarColor: Color(0xFF151515),
    systemNavigationBarDividerColor: Color(0xFF151515),
    statusBarIconBrightness: Brightness.light,
    systemNavigationBarIconBrightness: Brightness.light,
  ));

  // Keep immersive fullscreen so system bars are hidden during control mode.
  await SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);
  // Lock app to landscape for a controller-style interface.
  await SystemChrome.setPreferredOrientations(const [
    DeviceOrientation.landscapeLeft,
    DeviceOrientation.landscapeRight,
  ]);
  runApp(const MecanosaurusApp());
}

class MecanosaurusApp extends StatelessWidget {
  const MecanosaurusApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Mecanosaurus Controller',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: const Color(0xFF0B7285)),
        textTheme: GoogleFonts.jetBrainsMonoTextTheme(),
      ),
      home: const ControlScreen(),
    );
  }
}

class ControlScreen extends StatefulWidget {
  const ControlScreen({super.key});

  @override
  State<ControlScreen> createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen> {
  static const String _targetDeviceName = 'ESP32-S3-DEVKITC-1-N16R8V';
  static const String _controlServiceUuid = '12345678-1234-1234-1234-1234567890ab';
  static const String _controlCharacteristicUuid = '12345678-1234-1234-1234-1234567890ac';

  final GlobalKey<ScaffoldState> _scaffoldKey = GlobalKey<ScaffoldState>();
  double _joyX = 0.0;
  double _joyY = 0.0;
  Offset _knobOffset = Offset.zero;
  double _omega = 0.0;
  double _sliderKnobX = 0.0;

  BluetoothAdapterState _adapterState = BluetoothAdapterState.unknown;
  BluetoothDevice? _connectedDevice;
  String _btStatus = 'Not connected';
  bool _isScanning = false;
  List<ScanResult> _matchingResults = <ScanResult>[];
  BluetoothCharacteristic? _controlCharacteristic;

  StreamSubscription<BluetoothAdapterState>? _adapterSub;
  StreamSubscription<bool>? _scanStateSub;
  StreamSubscription<List<ScanResult>>? _scanResultsSub;
  StreamSubscription<BluetoothConnectionState>? _connectionStateSub;

  @override
  void initState() {
    super.initState();
    _adapterSub = FlutterBluePlus.adapterState.listen((state) {
      if (!mounted) return;
      setState(() {
        _adapterState = state;
      });
    });

    _scanStateSub = FlutterBluePlus.isScanning.listen((scanning) {
      if (!mounted) return;
      setState(() {
        _isScanning = scanning;
      });
    });

    _scanResultsSub = FlutterBluePlus.scanResults.listen((results) {
      final filtered = results.where((r) {
        final advertisedName = r.advertisementData.advName;
        final platformName = r.device.platformName;
        final hasControlService = r.advertisementData.serviceUuids.any(
          (uuid) => uuid.str128.toLowerCase() == _controlServiceUuid,
        );
        return advertisedName == _targetDeviceName ||
            platformName == _targetDeviceName ||
            hasControlService;
      }).toList();

      if (!mounted) return;
      setState(() {
        _matchingResults = filtered;
      });
    });
  }

  @override
  void dispose() {
    _adapterSub?.cancel();
    _scanStateSub?.cancel();
    _scanResultsSub?.cancel();
    _connectionStateSub?.cancel();
    super.dispose();
  }

  Future<void> _scanForEsp32() async {
    if (_adapterState != BluetoothAdapterState.on) {
      setState(() {
        _btStatus = 'Bluetooth is off. Enable it and try again.';
      });
      return;
    }

    setState(() {
      _matchingResults = <ScanResult>[];
      _btStatus = 'Scanning for $_targetDeviceName...';
    });

    try {
      await FlutterBluePlus.stopScan();
      await FlutterBluePlus.startScan(
        timeout: const Duration(seconds: 8),
        withNames: const <String>[_targetDeviceName],
      );

      if (!mounted) return;
      setState(() {
        if (_matchingResults.isEmpty) {
          _btStatus = 'Device not found. Keep ESP32 powered and advertising BLE.';
        } else {
          _btStatus = 'Device found. Tap Connect.';
        }
      });
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _btStatus = 'Scan failed: $e';
      });
    }
  }

  Future<void> _connectToDevice(BluetoothDevice device) async {
    try {
      await FlutterBluePlus.stopScan();
      await _connectionStateSub?.cancel();

      try {
        await device.connect(
          timeout: const Duration(seconds: 12),
          autoConnect: false,
        );
      } catch (e) {
        final msg = e.toString().toLowerCase();
        if (!msg.contains('already connected')) {
          rethrow;
        }
      }

      final services = await device.discoverServices();
      BluetoothCharacteristic? control;
      for (final service in services) {
        if (service.uuid.str128.toLowerCase() == _controlServiceUuid) {
          for (final characteristic in service.characteristics) {
            if (characteristic.uuid.str128.toLowerCase() ==
                _controlCharacteristicUuid) {
              control = characteristic;
              break;
            }
          }
        }
      }

      _connectionStateSub = device.connectionState.listen((state) {
        if (!mounted) return;
        setState(() {
          if (state == BluetoothConnectionState.connected) {
            _connectedDevice = device;
            _btStatus =
                'Connected to ${device.platformName.isEmpty ? _targetDeviceName : device.platformName}';
          } else if (state == BluetoothConnectionState.disconnected) {
            _connectedDevice = null;
            _controlCharacteristic = null;
            _btStatus = 'Disconnected';
          }
        });
      });

      if (!mounted) return;
      setState(() {
        _connectedDevice = device;
        _controlCharacteristic = control;
        _btStatus = control == null
            ? 'Connected, but control characteristic not found.'
            : 'Connected to ${device.platformName.isEmpty ? _targetDeviceName : device.platformName}';
      });

      if (control != null) {
        await _sendOmegaToEsp(_omega);
      }
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _btStatus = 'Connection failed: $e';
      });
    }
  }

  Future<void> _disconnectDevice() async {
    final device = _connectedDevice;
    if (device == null) return;

    try {
      await device.disconnect();
      if (!mounted) return;
      setState(() {
        _connectedDevice = null;
        _controlCharacteristic = null;
        _btStatus = 'Disconnected';
      });
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _btStatus = 'Disconnect failed: $e';
      });
    }
  }

  Future<void> _sendOmegaToEsp(double value) async {
    final characteristic = _controlCharacteristic;
    if (characteristic == null) return;

    try {
      final payload = utf8.encode('OMEGA:${value.toStringAsFixed(3)}');
      if (characteristic.properties.writeWithoutResponse) {
        await characteristic.write(payload, withoutResponse: true);
      } else if (characteristic.properties.write) {
        await characteristic.write(payload, withoutResponse: false);
      }
    } catch (_) {
      // Keep UI responsive even when a single BLE write fails.
    }
  }

  void _handleJoyPan(Offset localPos) {
    final delta = localPos - const Offset(110, 110);
    const maxTravel = 88.0;
    final clamped = delta.distance > maxTravel
        ? delta / delta.distance * maxTravel
        : delta;
    setState(() {
      _knobOffset = clamped;
      _joyX = clamped.dx / maxTravel;
      _joyY = -clamped.dy / maxTravel;
    });
  }

  void _handleJoyEnd() {
    setState(() {
      _knobOffset = Offset.zero;
      _joyX = 0.0;
      _joyY = 0.0;
    });
  }

  void _handleSliderEnd() {
    setState(() {
      _sliderKnobX = 0.0;
      _omega = 0.0;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      key: _scaffoldKey,
      appBar: AppBar(
        primary: false,
        backgroundColor: const Color(0xFF151515),
        bottom: const PreferredSize(
          preferredSize: Size.fromHeight(1),
          child: Divider(
            height: 1,
            thickness: 1,
            color: Color(0xFF343434),
          ),
        ),
        title: const Text(
          'Mecanosaurus',
          style: TextStyle(
            color: Color(0xFFe63416),
            fontWeight: FontWeight.w700,
          ),
        ),
        actions: [
          Padding(
            padding: const EdgeInsets.fromLTRB(8, 8, 15, 8),
            child: GestureDetector(
              onTap: () {
                _scaffoldKey.currentState?.openEndDrawer();
              },
              child: Image.asset(
                'assets/Samo-logo.png',
                height: 40,
                width: 40,
              ),
            ),
          ),
        ],
      ),
      body: Container(
        color: const Color(0xFF151515),
        child: Row(
          children: [
            Expanded(
              child: Column(
                children: [
                  Expanded(
                    flex: 1,
                    child: Row(
                      children: [
                        Expanded(
                          child: Column(
                            children: [
                              Expanded(
                                child: Center(
                                  child: Text(
                                    'X',
                                    style: TextStyle(
                                      color: Color(0xFFe63416),
                                      fontSize: 20,
                                    ),
                                  ),
                                ),
                              ),
                              Container(height: 1, color: const Color(0xFF343434)),
                              Expanded(
                                child: Center(
                                  child: Text(
                                    _joyX.toStringAsFixed(3),
                                    style: const TextStyle(
                                      color: Color(0xFFd5d5d5),
                                      fontSize: 20,
                                    ),
                                  ),
                                ),
                              ),
                            ],
                          ),
                        ),
                        Container(width: 1, color: const Color(0xFF343434)),
                        Expanded(
                          child: Column(
                            children: [
                              Expanded(
                                child: Center(
                                  child: Text(
                                    'Y',
                                    style: TextStyle(
                                      color: Color(0xFFe63416),
                                      fontSize: 20,
                                    ),
                                  ),
                                ),
                              ),
                              Container(height: 1, color: const Color(0xFF343434)),
                              Expanded(
                                child: Center(
                                  child: Text(
                                    _joyY.toStringAsFixed(3),
                                    style: const TextStyle(
                                      color: Color(0xFFd5d5d5),
                                      fontSize: 20,
                                    ),
                                  ),
                                ),
                              ),
                            ],
                          ),
                        ),
                        Container(width: 1, color: const Color(0xFF343434)),
                        Expanded(
                          child: Column(
                            children: [
                              Expanded(
                                child: Center(
                                  child: Text(
                                    'ω',
                                    style: TextStyle(
                                      color: Color(0xFFe63416),
                                      fontSize: 20,
                                    ),
                                  ),
                                ),
                              ),
                              Container(height: 1, color: const Color(0xFF343434)),
                              Expanded(
                                child: Center(
                                  child: Text(
                                    _omega.toStringAsFixed(3),
                                    style: const TextStyle(
                                      color: Color(0xFFd5d5d5),
                                      fontSize: 20,
                                    ),
                                  ),
                                ),
                              ),
                            ],
                          ),
                        ),
                      ],
                    ),
                  ),
                  Container(height: 1, color: const Color(0xFF343434)),
                  Expanded(
                    flex: 2,
                    child: Center(
                      child: LayoutBuilder(
                        builder: (context, constraints) {
                          final trackW = constraints.maxWidth * 0.75;
                          final halfW = trackW / 2;
                          return GestureDetector(
                            onPanUpdate: (d) {
                              final dx = (d.localPosition.dx - halfW)
                                  .clamp(-halfW, halfW);
                              final nextOmega = -(dx / halfW);
                              setState(() {
                                _sliderKnobX = dx;
                                _omega = nextOmega;
                              });
                              unawaited(_sendOmegaToEsp(nextOmega));
                            },
                            onPanEnd: (_) {
                              _handleSliderEnd();
                              unawaited(_sendOmegaToEsp(0.0));
                            },
                            child: SizedBox(
                              width: trackW,
                              height: 44,
                              child: Stack(
                                alignment: Alignment.center,
                                children: [
                                  Container(
                                    height: 1,
                                    color: const Color(0xFF343434),
                                  ),
                                  Transform.translate(
                                    offset: Offset(_sliderKnobX, 0),
                                    child: Container(
                                      width: 44,
                                      height: 44,
                                      decoration: BoxDecoration(
                                        shape: BoxShape.circle,
                                        color: const Color(0xFFe63416),
                                        border: Border.all(
                                          color: const Color(0xFF343434),
                                          width: 2,
                                        ),
                                      ),
                                    ),
                                  ),
                                ],
                              ),
                            ),
                          );
                        },
                      ),
                    ),
                  ),
                ],
              ),
            ),
            Container(width: 1, color: const Color(0xFF343434)),
            Expanded(
              child: Center(
                child: GestureDetector(
                  onPanUpdate: (d) => _handleJoyPan(d.localPosition),
                  onPanEnd: (_) => _handleJoyEnd(),
                  child: Container(
                    width: 220,
                    height: 220,
                    decoration: BoxDecoration(
                      shape: BoxShape.circle,
                      color: const Color(0xFF1D1D1D),
                      border: Border.all(color: const Color(0xFF343434), width: 2),
                    ),
                    child: Stack(
                      alignment: Alignment.center,
                      children: [
                        Transform.translate(
                          offset: _knobOffset,
                          child: Container(
                            width: 44,
                            height: 44,
                            decoration: BoxDecoration(
                              shape: BoxShape.circle,
                              color: const Color(0xFFe63416),
                              border: Border.all(
                                color: const Color(0xFF343434),
                                width: 1.5,
                              ),
                              boxShadow: const [
                                BoxShadow(
                                  color: Color(0x55000000),
                                  blurRadius: 8,
                                  offset: Offset(0, 3),
                                ),
                              ],
                            ),
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
      endDrawer: Drawer(
        backgroundColor: const Color(0xFFe63416),
        child: ListView(
          padding: EdgeInsets.zero,
          children: [
            const DrawerHeader(
              decoration: BoxDecoration(
                color: Color(0xFFe63416),
              ),
              child: Text(
                'Bluetooth Settings',
                style: TextStyle(
                  color: Color(0xFF151515),
                  fontSize: 18,
                  fontWeight: FontWeight.w700,
                ),
              ),
            ),
            ListTile(
              title: const Text(
                'Target device',
                style: TextStyle(color: Color(0xFF151515), fontWeight: FontWeight.w700),
              ),
              subtitle: const Text(
                _targetDeviceName,
                style: TextStyle(color: Color(0xFF151515)),
              ),
            ),
            ListTile(
              title: const Text(
                'Adapter',
                style: TextStyle(color: Color(0xFF151515), fontWeight: FontWeight.w700),
              ),
              subtitle: Text(
                _adapterState.name,
                style: const TextStyle(color: Color(0xFF151515)),
              ),
            ),
            ListTile(
              title: const Text(
                'Status',
                style: TextStyle(color: Color(0xFF151515), fontWeight: FontWeight.w700),
              ),
              subtitle: Text(
                _btStatus,
                style: const TextStyle(color: Color(0xFF151515)),
              ),
            ),
            ListTile(
              title: const Text(
                'Control channel',
                style: TextStyle(color: Color(0xFF151515), fontWeight: FontWeight.w700),
              ),
              subtitle: Text(
                _controlCharacteristic == null ? 'Not ready' : 'Ready',
                style: const TextStyle(color: Color(0xFF151515)),
              ),
            ),
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
              child: FilledButton(
                style: FilledButton.styleFrom(
                  backgroundColor: const Color(0xFF151515),
                  foregroundColor: const Color(0xFFEAEAEA),
                ),
                onPressed: _isScanning ? null : _scanForEsp32,
                child: Text(_isScanning ? 'Scanning...' : 'Scan for ESP32'),
              ),
            ),
            if (_connectedDevice != null)
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                child: OutlinedButton(
                  style: OutlinedButton.styleFrom(
                    foregroundColor: const Color(0xFF151515),
                    side: const BorderSide(color: Color(0xFF151515), width: 1.5),
                  ),
                  onPressed: _disconnectDevice,
                  child: const Text('Disconnect'),
                ),
              ),
            const Divider(color: Color(0xFF151515), thickness: 1),
            ..._matchingResults.map((result) {
              final advertisedName = result.advertisementData.advName;
              final name = advertisedName.isNotEmpty
                  ? advertisedName
                  : (result.device.platformName.isNotEmpty
                      ? result.device.platformName
                      : 'Unnamed');
              final isConnected = _connectedDevice?.remoteId == result.device.remoteId;

              return ListTile(
                title: Text(
                  name,
                  style: const TextStyle(color: Color(0xFF151515), fontWeight: FontWeight.w700),
                ),
                subtitle: Text(
                  result.device.remoteId.str,
                  style: const TextStyle(color: Color(0xFF151515)),
                ),
                trailing: FilledButton(
                  style: FilledButton.styleFrom(
                    backgroundColor: const Color(0xFF151515),
                    foregroundColor: const Color(0xFFEAEAEA),
                  ),
                  onPressed: isConnected ? null : () => _connectToDevice(result.device),
                  child: Text(isConnected ? 'Connected' : 'Connect'),
                ),
              );
            }),
          ],
        ),
      ),
    );
  }
}
