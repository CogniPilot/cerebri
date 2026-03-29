# AGENTS.md

Do not treat the repository root as the application root.

Vehicle and platform applications live in their own top-level folders such as
`rdd2/`.

When working in a vehicle or platform folder, look for `<vehicle>/spec/` and
adhere to the policies defined there. Treat that local `spec/` directory as the
source of truth for that vehicle.

There is intentionally no root `west.yml`.
