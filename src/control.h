#pragma once

// Returns false when robot is fallen and still past FALL_ANGLE (caller should return early).
bool controlTick(float dt);
void velLoopUpdate();
void deadManCheck();
void printDiagnostics();
