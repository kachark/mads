
use plotters::prelude::*;

pub fn plot_trajectory() -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new("images/trajectory.png", (640, 480)).into_drawing_area();
    root.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&root)
        .caption("y=x^2", ("sans-serif", 50).into_font())
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(-1f32..1f32, -0.1f32..1f32)?;

    chart.configure_mesh().draw()?;

    chart
        .draw_series(LineSeries::new(
            (-50..=50).map(|x| x as f32 / 50.0).map(|x| (x, x * x)),
            &RED,
        ))?
        .label("y = x^2")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    Ok(())
}

fn plot2() {
    let root_area = BitMapBackend::new("images/2.5.png", (600, 400))
        .into_drawing_area();
root_area.fill(&WHITE).unwrap();

let mut ctx = ChartBuilder::on(&root_area)
    .set_label_area_size(LabelAreaPosition::Left, 40)
    .set_label_area_size(LabelAreaPosition::Bottom, 40)
    .caption("Line Plot Demo", ("sans-serif", 40))
    .build_cartesian_2d(-10..10, 0..100)
    .unwrap();

ctx.configure_mesh().draw().unwrap();

ctx.draw_series(
    LineSeries::new((-10..=10).map(|x| (x, x* x)), &GREEN)
).unwrap();

}

