{
  op: "message",
  topic: "/plot_a",
  datatype: "webviz_msgs/2DPlotMsg",
  receiveTime: { sec: 1532375120, nsec: 317760607 },
  message: {
    type: "webviz_msgs/2DPlotMsg",
    title: "This is Plot A",
    xAxisLabel: "This is my X axis label",
    yAxisLabel: "This is my Y axis label",
    lines: [
      { label: "solid-line", borderColor: "red", backgroundColor: "red", data: [{ x: 0, y: 0 }, { x: 5, y: 5 }] },
      {
        label: "dashed-line",
        borderDash: [5, 5],
        borderColor: "pink",
        backgroundColor: "pink",
        data: [{ x: 1, y: 0.5 }, { x: 5, y: 3.5 }],
      },
    ],
    points: [
      {
        label: "circle-point",
        pointBackgroundColor: "blue",
        data: [{ x: 1.5, y: 2.5 }, { x: 3, y: 4 }, { x: 4, y: 3.5 }],
      },
      {
        label: "cross-point",
        pointBackgroundColor: "teal",
        pointBorderColor: "teal",
        pointStyle: "crossRot",
        pointRadius: 10,
        data: [{ x: 2, y: 1 }, { x: 4, y: 1 }],
      },
    ],
    polygons: [
      {
        label: "polygon",
        borderColor: "lightgray",
        backgroundColor: "lightgray",
        data: [{ x: 3, y: 1 }, { x: 4, y: 2 }, { x: 4.5, y: 1.5 }],
      },
    ],
  },
}