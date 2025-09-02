# Pull Request

## Description
<!-- Brief description of what this PR does -->

## Type of Change
<!-- Mark the relevant option with an "x" -->
- [ ] 🐛 Bug fix (non-breaking change which fixes an issue)
- [ ] ✨ New feature (non-breaking change which adds functionality)
- [ ] 💥 Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] 📚 Documentation update
- [ ] 🔧 Configuration change
- [ ] 🧹 Code cleanup/refactoring

## ROS 2 Components Affected
<!-- Check all that apply -->
- [ ] 🤖 Vehicle Description (URDF, sensors)
- [ ] 🗺️ Navigation (Nav2, path planning)
- [ ] 📍 Localization (GPS, IMU, EKF)
- [ ] 🎮 Simulation (Gazebo, worlds)
- [ ] 🎯 SLAM (Cartographer)
- [ ] 🔧 Control systems
- [ ] 🚀 Launch files
- [ ] 📦 Dependencies

## Testing
<!-- Describe how you tested this change -->
- [ ] ✅ Local build successful (`colcon build`)
- [ ] ✅ CI pipeline passes locally (`./ci/run.sh`)
- [ ] ✅ Integration tests pass
- [ ] ✅ Simulation testing completed
- [ ] ✅ Real hardware testing (if applicable)

### Test Commands Used
```bash
# List the commands you used to test this change
# Example:
# ./ci/run.sh build-and-test
# ros2 launch vehicle_bringup navigation.launch.py
```

## CI/CD Pipeline
<!-- The GitHub Actions CI will automatically run -->
- [ ] ✅ Linting passes (`ruff check`)
- [ ] ✅ Build tests pass (multiple ROS distros)
- [ ] ✅ Integration tests pass
- [ ] ✅ Documentation builds successfully

## Screenshots/Videos
<!-- If applicable, add screenshots or videos showing the changes -->

## Related Issues
<!-- Link any related issues -->
Fixes #(issue number)

## Checklist
- [ ] 📝 My code follows the project's style guidelines
- [ ] 🔍 I have performed a self-review of my code
- [ ] 💬 I have commented my code, particularly in hard-to-understand areas
- [ ] 📚 I have made corresponding changes to the documentation
- [ ] ⚠️ My changes generate no new warnings
- [ ] 🧪 I have added tests that prove my fix is effective or that my feature works
- [ ] 🔄 New and existing unit tests pass locally with my changes
- [ ] 📦 Any dependent changes have been merged and published

## Additional Notes
<!-- Any additional information about this PR -->

---

**Note**: This PR will be tested using the same Dagger CI pipeline that runs locally. The GitHub Actions workflow uses identical build environments and test procedures for consistency.
