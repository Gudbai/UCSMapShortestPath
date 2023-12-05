using Microsoft.Maps.MapControl.WPF;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.NetworkInformation;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace UCSTest
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 
    //public static class LinqHelper
    //{
    //    public static int TextToNumber(string text)
    //    {
    //        int sum = 0;
    //        foreach (char c in text)
    //        {
    //            sum = sum * 26 + c - 'A' + 1;
    //        }
    //        return sum;
    //    }
    //}

    class Graph
    {
        private Dictionary<string, List<(string, int)>> graph = new Dictionary<string, List<(string, int)>>();
        private Dictionary<(string, string), int> edgeCosts = new Dictionary<(string, string), int>();

        public void AddEdge(string source, string destination, int cost)
        {
            if (!graph.ContainsKey(source))
            {
                graph[source] = new List<(string, int)>();
            }
            graph[source].Add((destination, cost));
            edgeCosts[(source, destination)] = cost;

            if (!graph.ContainsKey(destination))
            {
                graph[destination] = new List<(string, int)>();
            }
        }

        public List<(string, int)> GetNeighbors(string node)
        {
            if (graph.ContainsKey(node))
            {
                return graph[node];
            }
            return new List<(string, int)>();
        }

        public int GetEdgeCost(string source, string destination)
        {
            return edgeCosts.ContainsKey((source, destination)) ? edgeCosts[(source, destination)] : 0;
        }
    }

    class UniformCostSearch
    {
        public static List<(List<string> path, int cost)> UCS(Graph graph, string start, string goal)
        {
            var priorityQueue = new PriorityQueue<(string, List<string>, int, int)>((a, b) => b.Item4.CompareTo(a.Item4));
            priorityQueue.Enqueue((start, new List<string> { start }, 0, 0));
            var routes = new List<(List<string> path, int cost)>();

            while (priorityQueue.Count > 0)
            {
                var (currentNode, path, currentCost, totalCost) = priorityQueue.Dequeue();

                if (currentNode == goal)
                {
                    routes.Add((path, currentCost));
                }

                foreach (var (neighbor, cost) in graph.GetNeighbors(currentNode))
                {
                    if (!path.Contains(neighbor))
                    {
                        var newPath = new List<string>(path) { neighbor };
                        var newCost = currentCost + cost;
                        var newTotalCost = totalCost + newCost;
                        priorityQueue.Enqueue((neighbor, newPath, newCost, newTotalCost));
                    }
                }
            }

            routes.Sort((a, b) => b.cost.CompareTo(a.cost));

            return routes;
        }

        public static void PrintPath(List<string> path, int currentCost, Graph graph, TextBox textBox, HashSet<string> printedPaths)
        {
            var pathKey = string.Join("->", path);
            if (!printedPaths.Contains(pathKey))
            {
                printedPaths.Add(pathKey);

                StringBuilder resultStringBuilder = new StringBuilder();

                if (printedPaths.Count == 1)
                {
                    textBox.Clear();
                    textBox.Text = "All possible routes sorted from best to worst:\n";
                }

                resultStringBuilder.Append("Route: ");
                for (int i = 0; i < path.Count - 1; i++)
                {
                    int edgeCost = graph.GetEdgeCost(path[i], path[i + 1]);
                    resultStringBuilder.Append($"{path[i]} ({edgeCost}) -> ");
                }
                resultStringBuilder.AppendLine($"{path[path.Count - 1]}");
                resultStringBuilder.AppendLine("Total cost: " + currentCost);
                resultStringBuilder.AppendLine();

                textBox.AppendText(resultStringBuilder.ToString());
            }
        }

        static Graph graph = new Graph();
    }

    class PriorityQueue<T>
    {
        private List<T> items;
        private Func<T, T, int> compareFunction;

        public int Count => items.Count;

        public PriorityQueue(Func<T, T, int> compareFunction)
        {
            this.items = new List<T>();
            this.compareFunction = compareFunction;
        }

        public void Enqueue(T item)
        {
            items.Add(item);
            int currentIndex = items.Count - 1;

            while (currentIndex > 0)
            {
                int parentIndex = (currentIndex - 1) / 2;
                if (compareFunction(items[currentIndex], items[parentIndex]) >= 0)
                {
                    break;
                }

                Swap(currentIndex, parentIndex);
                currentIndex = parentIndex;
            }
        }

        public T Dequeue()
        {
            if (items.Count == 0)
            {
                throw new InvalidOperationException("Queue is empty.");
            }

            T item = items[0];
            int lastIndex = items.Count - 1;
            items[0] = items[lastIndex];
            items.RemoveAt(lastIndex);

            int currentIndex = 0;
            while (true)
            {
                int leftChildIndex = currentIndex * 2 + 1;
                int rightChildIndex = currentIndex * 2 + 2;
                int nextIndex = currentIndex;

                if (leftChildIndex < lastIndex && compareFunction(items[leftChildIndex], items[nextIndex]) < 0)
                {
                    nextIndex = leftChildIndex;
                }
                if (rightChildIndex < lastIndex && compareFunction(items[rightChildIndex], items[nextIndex]) < 0)
                {
                    nextIndex = rightChildIndex;
                }
                if (nextIndex == currentIndex)
                {
                    break;
                }

                Swap(currentIndex, nextIndex);
                currentIndex = nextIndex;
            }

            return item;
        }

        private void Swap(int indexA, int indexB)
        {
            T temp = items[indexA];
            items[indexA] = items[indexB];
            items[indexB] = temp;
        }
    }

    public partial class MainWindow : Window
    {
        private Dictionary<string, Location> coordinatesDictionary = new Dictionary<string, Location>();

        public MainWindow()
        {
            InitializeComponent();
            MainMap.Mode = new AerialMode(true);
            MainMap.Focus();
            MainMap.Culture = "en";
        }

        private double CalculateHaversineDistance(Location location1, Location location2)
        {
            const double EarthRadius = 6371000; // Earth radius in kilometers

            double dLat = ToRadians(location2.Latitude - location1.Latitude);
            double dLon = ToRadians(location2.Longitude - location1.Longitude);

            double a = Math.Sin(dLat / 2) * Math.Sin(dLat / 2) +
                       Math.Cos(ToRadians(location1.Latitude)) * Math.Cos(ToRadians(location2.Latitude)) *
                       Math.Sin(dLon / 2) * Math.Sin(dLon / 2);

            double c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));

            return EarthRadius * c;
        }

        private double ToRadians(double degrees)
        {
            return degrees * Math.PI / 180;
        }

        private void MainMap_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            e.Handled = true;

            // Remove existing line and label associated with the pin
            string selectedLetter = CboxSelected.Text;
            var linesToRemove = connectionLines.Where(entry => entry.Key.Contains(selectedLetter)).ToList();
            foreach (var entry in linesToRemove)
            {
                MainMap.Children.Remove(entry.Value);

                // Remove the label associated with the removed line
                if (distanceLabels.TryGetValue(entry.Key, out var labelToRemove))
                {
                    MainMap.Children.Remove(labelToRemove);
                    distanceLabels.Remove(entry.Key);
                }

                connectionLines.Remove(entry.Key);
            }

            // Remove pin if it exists
            var pushpinToDelete = MainMap.Children.OfType<Pushpin>().FirstOrDefault(pin => pin.Tag?.ToString() == selectedLetter);
            if (pushpinToDelete != null)
            {
                MainMap.Children.Remove(pushpinToDelete);
            }

            // Get mouselocation on map
            Point mousePosition = e.GetPosition(this);
            Location pinLocation = MainMap.ViewportPointToLocation(mousePosition);

            Pushpin pin = new Pushpin();
            pin.Tag = selectedLetter; // Give it a tag
            pin.Location = pinLocation;
            pin.Content = selectedLetter; // Set the content of the pin to the selected letter

            // Set textbox.text to longitude and latitude
            TxbCoordinatesLong.Text = pinLocation.Longitude.ToString();
            TxbCoordinatesLat.Text = pinLocation.Latitude.ToString();

            // Save the coordinates to the dictionary using ComboBox item content
            if (CboxSelected.SelectedItem is ComboBoxItem selectedItem)
            {
                string selectedContent = selectedItem.Content.ToString();
                coordinatesDictionary[selectedContent] = pinLocation;
            }

            MainMap.Children.Add(pin);
        }

        private void CboxSelected_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            // Display the saved coordinates when the ComboBox selection changes
            if (TxbCoordinatesLong != null && TxbCoordinatesLat != null)
            {
                if (CboxSelected.SelectedItem is ComboBoxItem selectedItem)
                {
                    string selectedContent = selectedItem.Content.ToString();

                    if (coordinatesDictionary.TryGetValue(selectedContent, out var coordinates))
                    {
                        TxbCoordinatesLong.Text = coordinates.Longitude.ToString();
                        TxbCoordinatesLat.Text = coordinates.Latitude.ToString();
                    }
                    else
                    {
                        TxbCoordinatesLong.Text = "0";
                        TxbCoordinatesLat.Text = "0";
                    }
                }
            }
        }

        private void BtnDelete_Click(object sender, RoutedEventArgs e)
        {
            if (CboxSelected.SelectedItem is ComboBoxItem selectedItem)
            {
                string selectedContent = selectedItem.Content.ToString();

                // Remove the coordinates from the dictionary
                coordinatesDictionary.Remove(selectedContent);

                // Remove the lines and labels
                var linesToRemove = connectionLines.Where(entry => entry.Key.Contains(selectedContent)).ToList();
                foreach (var entry in linesToRemove)
                {
                    MainMap.Children.Remove(entry.Value);

                    // Remove the label associated with the removed line
                    if (distanceLabels.TryGetValue(entry.Key, out var labelToRemove))
                    {
                        MainMap.Children.Remove(labelToRemove);
                        distanceLabels.Remove(entry.Key);
                    }

                    connectionLines.Remove(entry.Key);
                }

                // Clear the TextBoxes
                TxbCoordinatesLong.Text = "0";
                TxbCoordinatesLat.Text = "0";

                // Remove the Pushpin from the map if it exists
                var pushpinToDelete = MainMap.Children.OfType<Pushpin>().FirstOrDefault(pin => pin.Tag?.ToString() == selectedContent);
                if (pushpinToDelete != null)
                {
                    MainMap.Children.Remove(pushpinToDelete);
                }
            }
        }

        private Dictionary<string, MapPolyline> connectionLines = new Dictionary<string, MapPolyline>();
        private Dictionary<string, TextBlock> distanceLabels = new Dictionary<string, TextBlock>();

        private void BtnDrawConnection_Click(object sender, RoutedEventArgs e)
        {
            string connectionInput = TxbConnectionInput.Text;

            if (!string.IsNullOrEmpty(connectionInput))
            {
                string[] connectionInputs = connectionInput.Split(new[] { '\r', '\n', ',' }, StringSplitOptions.RemoveEmptyEntries);

                // Create HashSets to store the connections from the current and previous inputs
                var currentConnections = new HashSet<string>();
                var previousConnections = new HashSet<string>(connectionLines.Keys);

                foreach (var input in connectionInputs)
                {
                    string[] connectionParts = input.Split(new[] { "->" }, StringSplitOptions.RemoveEmptyEntries);

                    if (connectionParts.Length == 2)
                    {
                        string startPoint = connectionParts[0].Trim();
                        string endPoint = connectionParts[1].Trim();

                        // Form the connection key
                        string connectionKey = $"{startPoint} -> {endPoint}";

                        currentConnections.Add(connectionKey);

                        // Check if pins with the specified letters exist
                        var startPin = MainMap.Children.OfType<Pushpin>().FirstOrDefault(pin => pin.Tag?.ToString() == startPoint);
                        var endPin = MainMap.Children.OfType<Pushpin>().FirstOrDefault(pin => pin.Tag?.ToString() == endPoint);

                        if (startPin != null && endPin != null)
                        {
                            Location startLocation = startPin.Location;
                            Location endLocation = endPin.Location;

                            // Remove existing line and label if they exist
                            if (connectionLines.TryGetValue(connectionKey, out var existingLine))
                            {
                                MainMap.Children.Remove(existingLine);
                                connectionLines.Remove(connectionKey);
                            }

                            if (distanceLabels.TryGetValue(connectionKey, out var existingLabel))
                            {
                                MainMap.Children.Remove(existingLabel);
                                distanceLabels.Remove(connectionKey);
                            }

                            // Draw a line between start and end locations
                            MapPolyline polyline = new MapPolyline();
                            polyline.Locations = new LocationCollection() { startLocation, endLocation };
                            polyline.Stroke = new SolidColorBrush(Colors.Red);
                            polyline.StrokeThickness = 2;

                            MainMap.Children.Add(polyline);

                            // Calculate distance using Haversine formula
                            double distance = CalculateHaversineDistance(startLocation, endLocation);

                            // Create a label to display the distance
                            TextBlock label = new TextBlock();
                            label.Foreground = Brushes.Blue;
                            label.FontWeight = FontWeights.Bold;
                            label.TextAlignment = TextAlignment.Center;

                            if (distance > 1000)
                            {
                                label.Text = $"{Math.Round(distance / 1000, 2)} km"; // Convert meters to kilometers with two decimal places
                            }
                            else
                            {
                                label.Text = $"{Math.Round(distance)} m"; // Display distance in meters
                            }

                            // Calculate the midpoint for the label
                            Location midPoint = new Location((startLocation.Latitude + endLocation.Latitude) / 2, (startLocation.Longitude + endLocation.Longitude) / 2);

                            // Set the position of the label
                            MapLayer.SetPosition(label, midPoint);

                            MainMap.Children.Add(label);

                            // Store the line and label in the dictionaries
                            connectionLines[connectionKey] = polyline;
                            distanceLabels[connectionKey] = label;
                        }
                        else
                        {
                            MessageBox.Show("One or both of the specified pins do not exist on the map.");
                        }
                    }
                    else
                    {
                        MessageBox.Show("Invalid connection input format. Please use 'A -> B' format.");
                    }
                }

                // Remove connections that are not present in the current input
                var keysToRemove = previousConnections.Except(currentConnections).ToList();
                foreach (var key in keysToRemove)
                {
                    var lineToRemove = connectionLines[key];
                    MainMap.Children.Remove(lineToRemove);
                    connectionLines.Remove(key);

                    // Remove the label associated with the removed line
                    if (distanceLabels.TryGetValue(key, out var labelToRemove))
                    {
                        MainMap.Children.Remove(labelToRemove);
                        distanceLabels.Remove(key);
                    }
                }
            }
            else
            {
                // Clear all connections if the input is empty
                foreach (var line in connectionLines.Values)
                {
                    MainMap.Children.Remove(line);
                }
                connectionLines.Clear();

                // Clear all distance labels if the input is empty
                foreach (var label in distanceLabels.Values)
                {
                    MainMap.Children.Remove(label);
                }
                distanceLabels.Clear();

                MessageBox.Show("Please enter a connection input.");
            }
        }

        private void BtnRunUCS_Click(object sender, RoutedEventArgs e)
        {
            // Check if there are at least two pins on the map
            if (MainMap.Children.OfType<Pushpin>().Count() < 2)
            {
                MessageBox.Show("Please add at least two pins on the map.");
                return;
            }

            // Get start and goal pins from ComboBoxes
            string startPin = CboxStart.Text;
            string endPin = CboxEnd.Text;

            // Check if both start and goal pins are selected
            if (string.IsNullOrEmpty(startPin) || string.IsNullOrEmpty(endPin))
            {
                MessageBox.Show("Please select both start and goal pins.");
                return;
            }

            // Run UCS algorithm
            RunUCS(startPin, endPin);
        }

        private void RunUCS(string start, string goal)
        {
            var graph = BuildGraphFromPins();
            var routes = UniformCostSearch.UCS(graph, CboxStart.Text, CboxEnd.Text);

            // Sort routes based on cost in ascending order
            routes.Sort((a, b) => a.cost.CompareTo(b.cost));

            // Find the best (shortest) path
            var bestPath = routes.FirstOrDefault();

            // Highlight the best path by changing the color to green
            HighlightBestPath(bestPath);

            var printedPaths = new HashSet<string>();
            foreach (var route in routes)
            {
                UniformCostSearch.PrintPath(route.path, route.cost, graph, TxbShortestPath, printedPaths);
            }
        }

        private void HighlightBestPath((List<string> path, int cost) bestPath)
        {
            if (bestPath != default)
            {
                // Iterate through each connection line and update the color
                foreach (var entry in connectionLines)
                {
                    // Get the start and end points of the line
                    string[] connectionParts = entry.Key.Split(new[] { "->" }, StringSplitOptions.RemoveEmptyEntries);
                    if (connectionParts.Length == 2)
                    {
                        string startTag = connectionParts[0].Trim();
                        string endTag = connectionParts[1].Trim();

                        // Check if the current connection is part of the best path
                        if (bestPath.path.Contains(startTag) && bestPath.path.Contains(endTag))
                        {
                            // Update the color of the line to green
                            entry.Value.Stroke = new SolidColorBrush(Colors.Green);

                            // You can also update other properties of the line, if needed
                            // entry.Value.StrokeThickness = 3; // for example
                        }
                    }
                }
            }
        }

        private Graph BuildGraphFromPins()
        {
            var graph = new Graph();

            // Check if there are any connection lines drawn
            if (connectionLines.Count > 0)
            {
                // Iterate through each connection line and add edges to the graph
                foreach (var entry in connectionLines)
                {
                    string[] connectionParts = entry.Key.Split(new[] { "->" }, StringSplitOptions.RemoveEmptyEntries);

                    if (connectionParts.Length == 2)
                    {
                        string startTag = connectionParts[0].Trim();
                        string endTag = connectionParts[1].Trim();

                        // Get the distance associated with the connection
                        double distance;
                        if (distanceLabels.TryGetValue(entry.Key, out var label))
                        {
                            // Parse the distance from the label text
                            string labelText = label.Text;
                            if (labelText.EndsWith("km"))
                            {
                                distance = double.Parse(labelText.Replace("km", "")) * 1000; // Convert km to meters
                            }
                            else if (labelText.EndsWith("m"))
                            {
                                distance = double.Parse(labelText.Replace("m", ""));
                            }
                            else
                            {
                                distance = 0;
                            }

                            // Use the new AddEdgeCost method to add the distance
                            graph.AddEdge(startTag, endTag, (int)distance);
                        }
                        else
                        {
                            distance = 0;
                        }

                        // Add an edge to the graph with the distance as the cost
                        graph.AddEdge(startTag, endTag, (int)distance);
                    }
                }
            }
            return graph;
        }
    }
}