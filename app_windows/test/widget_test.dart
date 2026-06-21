import 'package:flutter_test/flutter_test.dart';

import 'package:app_windows/main.dart';

void main() {
  testWidgets('Control panel renders key widgets', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MecanosaurusApp(enableGamepad: false),
    );

    expect(find.text('Mecanosaurus'), findsOneWidget);
    expect(find.text('Nie polaczono z RPI'), findsOneWidget);
    expect(find.text('X'), findsOneWidget);
    expect(find.text('Y'), findsOneWidget);
  });
}
