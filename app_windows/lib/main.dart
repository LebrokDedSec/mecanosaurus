import 'dart:async';
import 'dart:convert';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:gamepads/gamepads.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

// ─── Data models ─────────────────────────────────────────────────────────────

class RpiStatus {
  const RpiStatus({
    this.connected = false,
    this.rssi = 0,
    this.battery = 0,
    this.charging = false,
    this.chargeCurrent = 0,
  });
  final bool connected;
  final int rssi;
  final double battery;
  final bool charging;
  final double chargeCurrent;

  RpiStatus copyWith({
    bool? connected,
    int? rssi,
    double? battery,
    bool? charging,
    double? chargeCurrent,
  }) {
    return RpiStatus(
      connected: connected ?? this.connected,
      rssi: rssi ?? this.rssi,
      battery: battery ?? this.battery,
      charging: charging ?? this.charging,
      chargeCurrent: chargeCurrent ?? this.chargeCurrent,
    );
  }
}

// ─── Entry point ──────────────────────────────────────────────────────────────

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  runApp(const MecanosaurusApp());
}

class MecanosaurusApp extends StatelessWidget {
  const MecanosaurusApp({super.key, this.enableGamepad = true});
  final bool enableGamepad;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Mecanosaurus Control Panel',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: const Color(0xFF0B7285)),
        textTheme: GoogleFonts.jetBrainsMonoTextTheme(),
        scaffoldBackgroundColor: const Color(0xFF151515),
      ),
      home: ControlPanel(enableGamepad: enableGamepad),
    );
  }
}

// ─── Main screen ──────────────────────────────────────────────────────────────

class ControlPanel extends StatefulWidget {
  const ControlPanel({super.key, this.enableGamepad = true});
  final bool enableGamepad;

  @override
  State<ControlPanel> createState() => _ControlPanelState();
}

class _ControlPanelState extends State<ControlPanel>
    with WidgetsBindingObserver {
  static const double _joystickSize = 140.0;
  static const double _joystickTravel = 54.0;
  static const double _sliderKnobSize = 34.0;

  double _joyX = 0.0;
  double _joyY = 0.0;
  double _omega = 0.0;

  RpiStatus _rpiStatus = const RpiStatus();
  WebSocketChannel? _wsChannel;
  StreamSubscription<dynamic>? _wsSub;
  Timer? _driveHeartbeat;
  String _rpiIp = '192.168.4.1';
  int _rpiPort = 8765;
  bool _connecting = false;

  Uint8List? _cameraFrame;
  List<Offset> _lidarPoints = const [];
  double _lidarRangeMax = 8.0;

  StreamSubscription<NormalizedGamepadEvent>? _gamepadSub;
  String _gamepadLabel = 'No gamepad';

  bool get _useGamepad =>
      widget.enableGamepad &&
      !kIsWeb &&
      defaultTargetPlatform == TargetPlatform.windows;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    if (_useGamepad) {
      _gamepadSub = Gamepads.normalizedEvents.listen(
        _handleGamepadEvent,
        onError: (_) {},
      );
    }
    _driveHeartbeat = Timer.periodic(const Duration(milliseconds: 70), (_) {
      _sendDrive();
    });
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    _driveHeartbeat?.cancel();
    _gamepadSub?.cancel();
    _wsSub?.cancel();
    _wsChannel?.sink.close();
    super.dispose();
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    if (state == AppLifecycleState.paused ||
        state == AppLifecycleState.detached) {
      _sendPayload('STOP');
      if (mounted) setState(() { _joyX = 0; _joyY = 0; _omega = 0; });
    }
  }

  Future<void> _connectToRpi() async {
    if (_connecting) return;
    setState(() => _connecting = true);
    try {
      await _wsSub?.cancel();
      _wsChannel?.sink.close();
      final channel = WebSocketChannel.connect(
        Uri.parse('ws://$_rpiIp:$_rpiPort'),
      );
      await channel.ready;
      _wsChannel = channel;
      _wsSub = channel.stream.listen(
        _handleRpiMessage,
        onError: (_) => _onWsDisconnected(),
        onDone: _onWsDisconnected,
      );
      if (!mounted) return;
      setState(() {
        _rpiStatus = _rpiStatus.copyWith(connected: true);
        _connecting = false;
      });
    } catch (_) {
      if (!mounted) return;
      setState(() { _rpiStatus = const RpiStatus(); _connecting = false; });
    }
  }

  void _disconnectFromRpi() {
    _wsSub?.cancel();
    _wsChannel?.sink.close();
    _wsChannel = null;
    _wsSub = null;
    if (mounted) {
      setState(() {
        _rpiStatus = const RpiStatus();
        _cameraFrame = null;
        _lidarPoints = const [];
      });
    }
  }

  void _onWsDisconnected() {
    if (!mounted) return;
    setState(() => _rpiStatus = const RpiStatus());
  }

  void _handleRpiMessage(dynamic raw) {
    if (!mounted) return;
    try {
      if (raw is String) {
        final msg = jsonDecode(raw) as Map<String, dynamic>;
        switch (msg['type'] as String?) {
          case 'status':
            setState(() {
              _rpiStatus = _rpiStatus.copyWith(
                connected: true,
                rssi: (msg['rssi'] as num?)?.toInt(),
                battery: (msg['battery'] as num?)?.toDouble(),
                charging: msg['charging'] as bool?,
                chargeCurrent: (msg['charge_current'] as num?)?.toDouble(),
              );
            });
          case 'lidar':
            final pts = msg['points'] as List<dynamic>?;
            if (pts != null) {
              final parsed = <Offset>[];
              for (final p in pts) {
                if (p is List && p.length >= 2) {
                  parsed.add(Offset(
                    (p[0] as num).toDouble(),
                    (p[1] as num).toDouble(),
                  ));
                }
              }
              setState(() {
                _lidarPoints = parsed;
                if (msg['range_max'] != null) {
                  _lidarRangeMax = (msg['range_max'] as num).toDouble();
                }
              });
            }
          case 'camera':
            final jpeg = msg['jpeg'] as String?;
            if (jpeg != null) {
              setState(() => _cameraFrame = base64Decode(jpeg));
            }
        }
      } else if (raw is List<int>) {
        setState(() => _cameraFrame = Uint8List.fromList(raw));
      }
    } catch (_) {}
  }

  void _sendPayload(String payload) => _wsChannel?.sink.add(payload);

  void _sendDrive() {
    if (_wsChannel == null) return;
    final x = _dz(_joyX);
    final y = _dz(_joyY);
    final w = _dz(_omega);
    _sendPayload(
        'DRIVE:${x.toStringAsFixed(3)},${y.toStringAsFixed(3)},${w.toStringAsFixed(3)}');
  }

  void _handleGamepadEvent(NormalizedGamepadEvent event) {
    final axis = event.axis;
    if (axis == null) {
      if (event.button == GamepadButton.touchpad && event.value > 0.5) {
        setState(() { _joyX = 0; _joyY = 0; _omega = 0; });
        _sendPayload('STOP');
      }
      return;
    }
    double? nx, ny, nw;
    if (axis == GamepadAxis.rightStickX) nx = _cl(event.value);
    else if (axis == GamepadAxis.rightStickY) ny = _cl(event.value);
    else if (axis == GamepadAxis.leftStickX) nw = _cl(event.value);
    if (nx == null && ny == null && nw == null) return;
    setState(() {
      if (nx != null) _joyX = nx;
      if (ny != null) _joyY = ny;
      if (nw != null) _omega = nw;
      _gamepadLabel = 'DualShock active';
    });
  }

  double _cl(double v) => v.clamp(-1.0, 1.0).toDouble();
  double _dz(double v) => v.abs() < 0.03 ? 0.0 : v;

  void _setDrive({double? joyX, double? joyY, double? omega}) {
    setState(() {
      if (joyX != null) _joyX = _cl(joyX);
      if (joyY != null) _joyY = _cl(joyY);
      if (omega != null) _omega = _cl(omega);
    });
    _sendDrive();
  }

  Future<void> _showConnectDialog() async {
    final ipCtrl = TextEditingController(text: _rpiIp);
    final portCtrl = TextEditingController(text: '$_rpiPort');
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: const Color(0xFF1D1D1D),
        title: const Text('Polacz z RPI',
            style: TextStyle(color: Color(0xFFEAEAEA))),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            TextField(
              controller: ipCtrl,
              style: const TextStyle(color: Color(0xFFEAEAEA)),
              decoration: const InputDecoration(
                labelText: 'Adres IP',
                labelStyle: TextStyle(color: Color(0xFF7E7E7E)),
              ),
            ),
            const SizedBox(height: 8),
            TextField(
              controller: portCtrl,
              style: const TextStyle(color: Color(0xFFEAEAEA)),
              keyboardType: TextInputType.number,
              decoration: const InputDecoration(
                labelText: 'Port',
                labelStyle: TextStyle(color: Color(0xFF7E7E7E)),
              ),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx, false),
            child: const Text('Anuluj',
                style: TextStyle(color: Color(0xFF7E7E7E))),
          ),
          FilledButton(
            style: FilledButton.styleFrom(
                backgroundColor: const Color(0xFFe63416)),
            onPressed: () => Navigator.pop(ctx, true),
            child: const Text('Polacz'),
          ),
        ],
      ),
    );
    if (confirmed == true && mounted) {
      setState(() {
        _rpiIp = ipCtrl.text.trim();
        _rpiPort = int.tryParse(portCtrl.text.trim()) ?? 8765;
      });
      await _connectToRpi();
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Column(
        children: [
          _StatusBar(
            rpiStatus: _rpiStatus,
            connecting: _connecting,
            onConnect: _rpiStatus.connected ? _disconnectFromRpi : _showConnectDialog,
          ),
          const Divider(height: 1, thickness: 1, color: Color(0xFF343434)),
          Expanded(
            flex: 3,
            child: Row(
              children: [
                Expanded(child: _CameraView(frame: _cameraFrame)),
                Container(width: 1, color: const Color(0xFF343434)),
                Expanded(
                  child: _LidarView(
                    points: _lidarPoints,
                    rangeMax: _lidarRangeMax,
                  ),
                ),
              ],
            ),
          ),
          const Divider(height: 1, thickness: 1, color: Color(0xFF343434)),
          SizedBox(
            height: 176,
            child: _ControlStrip(
              joyX: _joyX,
              joyY: _joyY,
              omega: _omega,
              useGamepad: _useGamepad,
              gamepadLabel: _gamepadLabel,
              joystickSize: _joystickSize,
              joystickTravel: _joystickTravel,
              sliderKnobSize: _sliderKnobSize,
              onJoyPan: (pos) {
                final delta = pos - Offset(_joystickSize / 2, _joystickSize / 2);
                final clamped = delta.distance > _joystickTravel
                    ? delta / delta.distance * _joystickTravel
                    : delta;
                _setDrive(
                  joyX: clamped.dx / _joystickTravel,
                  joyY: -clamped.dy / _joystickTravel,
                );
              },
              onJoyEnd: () => _setDrive(joyX: 0, joyY: 0),
              onSliderPan: (dx, halfW) => _setDrive(omega: dx / halfW),
              onSliderEnd: () => _setDrive(omega: 0),
            ),
          ),
        ],
      ),
    );
  }
}

// ─── Status bar ───────────────────────────────────────────────────────────────

class _StatusBar extends StatelessWidget {
  const _StatusBar({
    required this.rpiStatus,
    required this.connecting,
    required this.onConnect,
  });
  final RpiStatus rpiStatus;
  final bool connecting;
  final VoidCallback onConnect;

  @override
  Widget build(BuildContext context) {
    return Container(
      height: 52,
      color: const Color(0xFF1A1A1A),
      padding: const EdgeInsets.symmetric(horizontal: 16),
      child: Row(
        children: [
          const Text('Mecanosaurus',
              style: TextStyle(
                  color: Color(0xFFe63416),
                  fontWeight: FontWeight.w700,
                  fontSize: 17)),
          const SizedBox(width: 16),
          Container(width: 1, height: 26, color: const Color(0xFF343434)),
          const SizedBox(width: 16),
          _WifiStatus(status: rpiStatus),
          const Spacer(),
          if (rpiStatus.connected) ...[
            _BatteryIndicator(
              percent: rpiStatus.battery,
              charging: rpiStatus.charging,
              chargeCurrent: rpiStatus.chargeCurrent,
            ),
            const SizedBox(width: 16),
            Container(width: 1, height: 26, color: const Color(0xFF343434)),
            const SizedBox(width: 16),
          ],
          _ConnectButton(
            connected: rpiStatus.connected,
            connecting: connecting,
            onPressed: onConnect,
          ),
        ],
      ),
    );
  }
}

class _WifiStatus extends StatelessWidget {
  const _WifiStatus({required this.status});
  final RpiStatus status;

  @override
  Widget build(BuildContext context) {
    if (!status.connected) {
      return const Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(Icons.wifi_off, color: Color(0xFF5E5E5E), size: 16),
          SizedBox(width: 6),
          Text('Nie polaczono z RPI',
              style: TextStyle(color: Color(0xFF5E5E5E), fontSize: 12)),
        ],
      );
    }
    final rssi = status.rssi;
    final Color sig = rssi >= -60
        ? const Color(0xFF4CAF50)
        : rssi >= -75
            ? const Color(0xFFFF9800)
            : const Color(0xFFe63416);
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Icon(Icons.wifi, color: sig, size: 18),
        const SizedBox(width: 8),
        Column(
          mainAxisAlignment: MainAxisAlignment.center,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text('RPI polaczone',
                style: TextStyle(color: Color(0xFFd5d5d5), fontSize: 11)),
            Text('$rssi dBm', style: TextStyle(color: sig, fontSize: 10)),
          ],
        ),
      ],
    );
  }
}

class _BatteryIndicator extends StatelessWidget {
  const _BatteryIndicator({
    required this.percent,
    required this.charging,
    required this.chargeCurrent,
  });
  final double percent;
  final bool charging;
  final double chargeCurrent;

  @override
  Widget build(BuildContext context) {
    final Color c = percent > 60
        ? const Color(0xFF4CAF50)
        : percent > 25
            ? const Color(0xFFFF9800)
            : const Color(0xFFe63416);
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Icon(charging ? Icons.battery_charging_full : Icons.battery_std,
            color: c, size: 20),
        const SizedBox(width: 6),
        Column(
          mainAxisAlignment: MainAxisAlignment.center,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('${percent.toStringAsFixed(0)}%',
                style: TextStyle(
                    color: c, fontSize: 13, fontWeight: FontWeight.w700)),
            if (charging)
              Text('Ladowanie  ${chargeCurrent.toStringAsFixed(1)} A',
                  style: const TextStyle(color: Color(0xFF4CAF50), fontSize: 10))
            else
              const Text('Rozladowanie',
                  style: TextStyle(color: Color(0xFF7E7E7E), fontSize: 10)),
          ],
        ),
      ],
    );
  }
}

class _ConnectButton extends StatelessWidget {
  const _ConnectButton({
    required this.connected,
    required this.connecting,
    required this.onPressed,
  });
  final bool connected;
  final bool connecting;
  final VoidCallback onPressed;

  @override
  Widget build(BuildContext context) {
    return FilledButton.icon(
      style: FilledButton.styleFrom(
        backgroundColor:
            connected ? const Color(0xFF2A2A2A) : const Color(0xFFe63416),
        foregroundColor: Colors.white,
        minimumSize: const Size(130, 36),
        padding: const EdgeInsets.symmetric(horizontal: 14),
      ),
      onPressed: connecting ? null : onPressed,
      icon: connecting
          ? const SizedBox(
              width: 14,
              height: 14,
              child: CircularProgressIndicator(
                  strokeWidth: 2, color: Colors.white),
            )
          : Icon(connected ? Icons.link_off : Icons.wifi, size: 16),
      label: Text(
        connecting ? 'Laczenie...' : connected ? 'Rozlacz' : 'Polacz z RPI',
        style: const TextStyle(fontSize: 12),
      ),
    );
  }
}

// ─── Camera view ──────────────────────────────────────────────────────────────

class _CameraView extends StatelessWidget {
  const _CameraView({this.frame});
  final Uint8List? frame;

  @override
  Widget build(BuildContext context) {
    return Container(
      color: const Color(0xFF0D0D0D),
      child: Stack(
        fit: StackFit.expand,
        children: [
          if (frame != null)
            Image.memory(frame!, fit: BoxFit.contain, gaplessPlayback: true)
          else
            const Center(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Icon(Icons.videocam_off, color: Color(0xFF2A2A2A), size: 52),
                  SizedBox(height: 12),
                  Text('Kamera - brak sygnalu',
                      style: TextStyle(color: Color(0xFF3A3A3A), fontSize: 13)),
                ],
              ),
            ),
          const Positioned(
            top: 8,
            left: 12,
            child: Text('KAMERA',
                style: TextStyle(
                    color: Color(0xFF3A3A3A),
                    fontSize: 10,
                    letterSpacing: 1.5)),
          ),
        ],
      ),
    );
  }
}

// ─── LiDAR view ───────────────────────────────────────────────────────────────

class _LidarView extends StatelessWidget {
  const _LidarView({required this.points, required this.rangeMax});
  final List<Offset> points;
  final double rangeMax;

  @override
  Widget build(BuildContext context) {
    return Container(
      color: const Color(0xFF0D0D0D),
      child: Stack(
        fit: StackFit.expand,
        children: [
          if (points.isEmpty)
            const Center(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Icon(Icons.radar, color: Color(0xFF2A2A2A), size: 52),
                  SizedBox(height: 12),
                  Text('LiDAR - brak danych',
                      style: TextStyle(color: Color(0xFF3A3A3A), fontSize: 13)),
                ],
              ),
            )
          else
            RepaintBoundary(
              child: CustomPaint(
                painter: _LidarPainter(points: points, rangeMax: rangeMax),
              ),
            ),
          const Positioned(
            top: 8,
            left: 12,
            child: Text('LIDAR',
                style: TextStyle(
                    color: Color(0xFF3A3A3A),
                    fontSize: 10,
                    letterSpacing: 1.5)),
          ),
        ],
      ),
    );
  }
}

class _LidarPainter extends CustomPainter {
  const _LidarPainter({required this.points, required this.rangeMax});
  final List<Offset> points;
  final double rangeMax;

  @override
  void paint(Canvas canvas, Size size) {
    final cx = size.width / 2;
    final cy = size.height / 2;
    final scale = math.min(size.width, size.height) / 2 / rangeMax;

    final gridPaint = Paint()
      ..color = const Color(0xFF1C1C1C)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1;
    for (int ring = 1; ring <= 4; ring++) {
      canvas.drawCircle(
          Offset(cx, cy), scale * rangeMax * ring / 4, gridPaint);
    }
    canvas.drawLine(Offset(cx, cy - scale * rangeMax),
        Offset(cx, cy + scale * rangeMax), gridPaint);
    canvas.drawLine(Offset(cx - scale * rangeMax, cy),
        Offset(cx + scale * rangeMax, cy), gridPaint);

    final ptPaint = Paint()
      ..color = const Color(0xFF00E5FF)
      ..style = PaintingStyle.fill;
    for (final p in points) {
      final sx = cx + p.dx * scale;
      final sy = cy - p.dy * scale;
      if (sx >= 0 && sx <= size.width && sy >= 0 && sy <= size.height) {
        canvas.drawCircle(Offset(sx, sy), 2, ptPaint);
      }
    }

    canvas.drawCircle(
        Offset(cx, cy),
        5,
        Paint()
          ..color = const Color(0xFFe63416)
          ..style = PaintingStyle.fill);

    final tp = TextPainter(
      text: TextSpan(
        text: '${rangeMax.toStringAsFixed(0)} m',
        style: const TextStyle(color: Color(0xFF5E5E5E), fontSize: 10),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
    tp.paint(canvas,
        Offset(cx + scale * rangeMax - tp.width - 4, cy - tp.height - 2));
  }

  @override
  bool shouldRepaint(_LidarPainter old) =>
      points != old.points || rangeMax != old.rangeMax;
}

// ─── Control strip ────────────────────────────────────────────────────────────

class _ControlStrip extends StatelessWidget {
  const _ControlStrip({
    required this.joyX,
    required this.joyY,
    required this.omega,
    required this.useGamepad,
    required this.gamepadLabel,
    required this.joystickSize,
    required this.joystickTravel,
    required this.sliderKnobSize,
    required this.onJoyPan,
    required this.onJoyEnd,
    required this.onSliderPan,
    required this.onSliderEnd,
  });
  final double joyX;
  final double joyY;
  final double omega;
  final bool useGamepad;
  final String gamepadLabel;
  final double joystickSize;
  final double joystickTravel;
  final double sliderKnobSize;
  final void Function(Offset) onJoyPan;
  final VoidCallback onJoyEnd;
  final void Function(double dx, double halfW) onSliderPan;
  final VoidCallback onSliderEnd;

  @override
  Widget build(BuildContext context) {
    return Container(
      color: const Color(0xFF151515),
      child: Row(
        children: [
          // ── X / Y / omega values ─────────────────────────────────────────
          SizedBox(
            width: 230,
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 12),
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  if (useGamepad)
                    Padding(
                      padding: const EdgeInsets.only(bottom: 6),
                      child: Text(gamepadLabel,
                          style: const TextStyle(
                              color: Color(0xFF5E5E5E),
                              fontSize: 10,
                              letterSpacing: 1)),
                    ),
                  Row(
                    children: [
                      _AxisValueCell(label: 'X', value: joyX),
                      const SizedBox(width: 6),
                      _AxisValueCell(label: 'Y', value: joyY),
                      const SizedBox(width: 6),
                      _AxisValueCell(label: 'ω', value: omega),
                    ],
                  ),
                ],
              ),
            ),
          ),
          Container(width: 1, color: const Color(0xFF343434)),
          // ── Rotation slider ──────────────────────────────────────────────
          Expanded(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 24),
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  const Text('OBROT',
                      style: TextStyle(
                          color: Color(0xFF5E5E5E),
                          fontSize: 10,
                          letterSpacing: 1.5)),
                  const SizedBox(height: 10),
                  LayoutBuilder(
                    builder: (context, constraints) {
                      final halfW = constraints.maxWidth / 2;
                      return GestureDetector(
                        onPanUpdate: useGamepad
                            ? null
                            : (d) {
                                final dx = (d.localPosition.dx - halfW)
                                    .clamp(-halfW, halfW);
                                onSliderPan(dx, halfW);
                              },
                        onPanEnd: useGamepad ? null : (_) => onSliderEnd(),
                        onPanCancel: useGamepad ? null : onSliderEnd,
                        child: SizedBox(
                          width: double.infinity,
                          height: sliderKnobSize,
                          child: Stack(
                            alignment: Alignment.center,
                            children: [
                              Container(
                                  height: 1,
                                  color: const Color(0xFF343434)),
                              Transform.translate(
                                offset: Offset(halfW * omega, 0),
                                child: Container(
                                  width: sliderKnobSize,
                                  height: sliderKnobSize,
                                  decoration: BoxDecoration(
                                    shape: BoxShape.circle,
                                    color: const Color(0xFFe63416),
                                    border: Border.all(
                                        color: const Color(0xFF343434),
                                        width: 2),
                                  ),
                                ),
                              ),
                            ],
                          ),
                        ),
                      );
                    },
                  ),
                  if (useGamepad)
                    const Padding(
                      padding: EdgeInsets.only(top: 6),
                      child: Text('Lewa galka X',
                          style: TextStyle(
                              color: Color(0xFF5E5E5E), fontSize: 10)),
                    ),
                ],
              ),
            ),
          ),
          Container(width: 1, color: const Color(0xFF343434)),
          // ── Joystick circle ──────────────────────────────────────────────
          SizedBox(
            width: joystickSize + 48,
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                const Text('RUCH',
                    style: TextStyle(
                        color: Color(0xFF5E5E5E),
                        fontSize: 10,
                        letterSpacing: 1.5)),
                const SizedBox(height: 8),
                GestureDetector(
                  onPanUpdate: useGamepad
                      ? null
                      : (d) => onJoyPan(d.localPosition),
                  onPanEnd: useGamepad ? null : (_) => onJoyEnd(),
                  onPanCancel: useGamepad ? null : onJoyEnd,
                  child: Container(
                    width: joystickSize,
                    height: joystickSize,
                    decoration: BoxDecoration(
                      shape: BoxShape.circle,
                      color: const Color(0xFF1D1D1D),
                      border:
                          Border.all(color: const Color(0xFF343434), width: 2),
                    ),
                    child: Stack(
                      alignment: Alignment.center,
                      children: [
                        Transform.translate(
                          offset: Offset(
                              joyX * joystickTravel, -joyY * joystickTravel),
                          child: Container(
                            width: 30,
                            height: 30,
                            decoration: BoxDecoration(
                              shape: BoxShape.circle,
                              color: const Color(0xFFe63416),
                              border: Border.all(
                                  color: const Color(0xFF343434), width: 1.5),
                              boxShadow: const [
                                BoxShadow(
                                    color: Color(0x55000000),
                                    blurRadius: 6,
                                    offset: Offset(0, 2)),
                              ],
                            ),
                          ),
                        ),
                        if (useGamepad)
                          const Positioned(
                            bottom: 10,
                            child: Text('Prawa galka',
                                style: TextStyle(
                                    color: Color(0xFF5E5E5E), fontSize: 10)),
                          ),
                      ],
                    ),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

// ─── Axis value cell ──────────────────────────────────────────────────────────

class _AxisValueCell extends StatelessWidget {
  const _AxisValueCell({required this.label, required this.value});
  final String label;
  final double value;

  @override
  Widget build(BuildContext context) {
    return Expanded(
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 8),
        decoration: BoxDecoration(
          color: const Color(0xFF1D1D1D),
          borderRadius: BorderRadius.circular(6),
          border: Border.all(color: const Color(0xFF343434)),
        ),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text(label,
                style: const TextStyle(
                    color: Color(0xFFe63416),
                    fontSize: 11,
                    fontWeight: FontWeight.w700)),
            const SizedBox(height: 4),
            Text(value.toStringAsFixed(3),
                style:
                    const TextStyle(color: Color(0xFFd5d5d5), fontSize: 13)),
          ],
        ),
      ),
    );
  }
}
