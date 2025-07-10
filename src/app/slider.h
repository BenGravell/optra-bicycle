#pragma once

float Slider(Rectangle bounds, float value, float minValue, float maxValue, const char* label) {
    // Draw the slider background
    DrawRectangleRec(bounds, GRAY);

    // Compute normalized value
    float normalized = (value - minValue) / (maxValue - minValue);
    float knobX = bounds.x + normalized * bounds.width;

    Rectangle knob = {knobX - 5, bounds.y - 5, 10, bounds.height + 10};
    bool isHovered = CheckCollisionPointRec(GetMousePosition(), knob);
    bool isActive = isHovered && IsMouseButtonDown(MOUSE_LEFT_BUTTON);

    if (isActive) {
        float mouseX = GetMouseX();
        mouseX = std::clamp(mouseX, bounds.x, bounds.x + bounds.width);
        normalized = (mouseX - bounds.x) / bounds.width;
        value = minValue + normalized * (maxValue - minValue);
    }

    // Draw the knob
    DrawRectangleRec(knob, isHovered ? DARKGRAY : LIGHTGRAY);
    DrawText(label, bounds.x, bounds.y - 20, 10, BLACK);
    char valText[32];
    snprintf(valText, sizeof(valText), "%.2f", value);
    DrawText(valText, bounds.x + bounds.width + 10, bounds.y, 10, BLACK);

    return value;
}


// // SLIDER
// // TODO use this for filtering the time index range shown in the tree
// // TODO make the slider keep holding onto the mouse while click & drag engaged

// int main(void) {
//     InitWindow(800, 600, "Raylib Slider Widget");
//     SetTargetFPS(60);

//     float sliderValue = 0.5f;
//     Rectangle sliderBounds = { 100, 100, 300, 10 };

//     while (!WindowShouldClose()) {
//         BeginDrawing();
//         ClearBackground(RAYWHITE);

//         sliderValue = Slider(sliderBounds, sliderValue, 0.0f, 1.0f, "Volume");

//         EndDrawing();
//     }

//     CloseWindow();
//     return 0;
// }