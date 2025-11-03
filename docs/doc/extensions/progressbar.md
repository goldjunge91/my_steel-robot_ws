# Progressbar Extension

The `pymdownx.progressbar` extension is a Python-Markdown plugin that allows you to create visually appealing progress bars directly in your Markdown content.

## Configuration

```yaml
# mkdocs.yml
markdown_extensions:
  - pymdownx.progressbar
```

## Syntax

You can also leverage `attr_list` to colorize the progress bar.

```markdown
[=50% ""]
[=75% "75%"]
[=95% "Awesome"]{: .success}
[=25% "25%"]{: .warning}
[=5% "5%"]{: .danger}
```

[=50% ""]
[=75% "75%"]
[=95% "Awesome"]{: .success}
[=25% "25%"]{: .warning}
[=5% "5%"]{: .danger}

## Basic Examples

### Simple Progress Bars

```markdown
[=0% "Starting"]
[=25% "Quarter Done"]
[=50% "Half Way"]
[=75% "Almost There"]
[=100% "Complete!"]
```

[=0% "Starting"]
[=25% "Quarter Done"]
[=50% "Half Way"]
[=75% "Almost There"]
[=100% "Complete!"]

### Without Labels

```markdown
[=20%]
[=40%]
[=60%]
[=80%]
[=100%]
```

[=20%]
[=40%]
[=60%]
[=80%]
[=100%]

## Styled Progress Bars

Using `attr_list` extension for custom styling:

```markdown
[=90% "Excellent"]{: .success}
[=70% "Good"]{: .info}
[=50% "Average"]{: .warning}
[=20% "Needs Work"]{: .danger}
```

[=90% "Excellent"]{: .success}
[=70% "Good"]{: .info}
[=50% "Average"]{: .warning}
[=20% "Needs Work"]{: .danger}

## Use Cases

### Project Status

## Development Progress

**Frontend Development**
[=85% "UI components complete, styling in progress"]

**Backend API**
[=95% "All endpoints implemented, testing remaining"]

**Database Schema**
[=100% "Complete"]

**Documentation**
[=60% "API docs done, user guide in progress"]

**Testing**
[=40% "Unit tests complete, integration tests started"]

### Learning Progress

## Course Completion

**Module 1: Introduction**
[=100% "Completed"]

**Module 2: Fundamentals**
[=100% "Completed"]

**Module 3: Advanced Topics**
[=75% "3 of 4 lessons complete"]

**Module 4: Projects**
[=25% "1 of 4 projects complete"]

**Final Assessment**
[=0% "Not started"]

### System Monitoring

## Server Status

**CPU Usage**
[=45% "45% - Normal"]{: .success}

**Memory Usage**
[=78% "78% - High"]{: .warning}

**Disk Space**
[=92% "92% - Critical"]{: .danger}

**Network I/O**
[=23% "23% - Low"]{: .success}

## Integration with Other Extensions

### With Admonitions

!!! info "Project Update"
    Current development progress:
    
    [=75% "3 of 4 milestones completed"]
    
    Expected completion: Next week

### With Tabs

=== "Development"
    [=80% "Backend complete, frontend 60%"]

=== "Testing"
    [=45% "Unit tests done, integration tests in progress"]

=== "Documentation"
    [=30% "API docs complete, user guides started"]

### With Details Blocks

/// details | Detailed Progress
    type: info

**Core Features**
[=85% "Most features implemented"]

**Bug Fixes**
[=70% "Major bugs resolved"]

**Performance**
[=60% "Optimization in progress"]
///

## Best Practices

1. **Use meaningful percentages**: Round to sensible values (avoid 73.7%, use 75%)
2. **Provide context**: Include descriptive labels when helpful
3. **Be consistent**: Use the same scale throughout your documentation
4. **Update regularly**: Keep progress bars current and accurate
5. **Group related items**: Organize similar progress indicators together

## Styling Options

The progress bars automatically adapt to the shadcn theme and support custom CSS classes:

```markdown
[=80% "Custom Styled"]{: .my-custom-progress}
```

Available built-in classes:
- `.success` - Green styling for completed/successful items
- `.info` - Blue styling for informational progress
- `.warning` - Yellow/orange styling for caution items
- `.danger` - Red styling for critical/error states

## Advanced Usage

### Multi-level Progress

## Overall Project Progress
[=65% "Phase 2 of 3 in progress"]

### Phase 1: Planning
[=100% "Complete"]

### Phase 2: Development
[=75% "Backend done, frontend 50%"]

### Phase 3: Deployment
[=0% "Not started"]

### Progress with Context

### Database Migration
[=45% "Processing table 3 of 7"]

**Status:** Migrating user_data table  
**Estimated Time:** 2 hours remaining  
**Records Processed:** 450,000 of 1,000,000